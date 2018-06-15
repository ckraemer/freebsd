#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/gpio.h>
#include <sys/conf.h>
#include <sys/stat.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>

#include <dev/gpio/gpiobusvar.h>

struct gpiointr_pin {
	bool		configured;
	gpio_pin_t	pin;
	int		intr_rid;
	struct resource	*intr_res;
	void		*intr_cookie;
};

struct gpiointr_softc {
	device_t		dev;
	device_t		pdev;
	struct cdev		*cdev;
	struct mtx		mtx;
	int			npins;
	struct gpiointr_pin	*pins;
	struct selinfo		selinfo;
	bool			intr_toogle;
	bool			active;
	unsigned int		counter;
};

struct gpiointr_cdevpriv {
	struct gpiointr_softc *sc;
	bool poll_en;
	bool intr_toogle;
	bool kq_stale;
};

static MALLOC_DEFINE(M_GPIOINTR, "gpiointr", "gpiointr device data");

static const char*	gpiointr_intr_mode_to_str(uint32_t);
static int		gpiointr_allocate_pin(struct gpiointr_softc*, int);
static int		gpiointr_release_pin(struct gpiointr_softc*, int);
static int		gpiointr_probe(device_t);
static int		gpiointr_attach(device_t);
static int		gpiointr_detach(device_t);
static void		gpiointr_interrupt_handler(void*);
static void		gpiointr_cdevpriv_dtor(void*);

static d_open_t		gpiointr_open;
static d_close_t	gpiointr_close;
static d_read_t		gpiointr_read;
static d_ioctl_t	gpiointr_ioctl;
static d_poll_t		gpiointr_poll;
static d_kqfilter_t	gpiointr_kqfilter;

static int		gpiointr_kqread(struct knote*, long);
static void		gpiointr_kqdetach(struct knote*);

static struct cdevsw gpiointr_cdevsw = {
	.d_version = D_VERSION,
	.d_open = gpiointr_open,
	.d_close = gpiointr_close,
	.d_read = gpiointr_read,
	.d_ioctl = gpiointr_ioctl,
	.d_poll = gpiointr_poll,
	.d_kqfilter = gpiointr_kqfilter
};

static struct filterops gpiointr_read_filterops = {
	.f_isfd =	true,
	.f_attach =	NULL,
	.f_detach =	gpiointr_kqdetach,
	.f_event =	gpiointr_kqread,
	.f_touch =	NULL
};

static const char*
gpiointr_intr_mode_to_str(uint32_t intr_mode)
{
	switch (intr_mode) {
	case GPIO_INTR_LEVEL_LOW:
		return "low level";
	case GPIO_INTR_LEVEL_HIGH:
		return "high level";
	case GPIO_INTR_EDGE_RISING:
		return "rising edge";
	case GPIO_INTR_EDGE_FALLING:
		return "falling edge";
	case GPIO_INTR_EDGE_BOTH:
		return "both edges";
	default:
		return "invalid mode";
	}
}

static int
gpiointr_allocate_pin(struct gpiointr_softc *sc, int pinnumber)
{
	uint32_t intr_mode;
	int err;

	intr_mode = sc->pins[pinnumber].pin->flags & GPIO_INTR_MASK;

	sc->pins[pinnumber].intr_res = gpio_alloc_intr_resource(sc->pins[pinnumber].pin->dev, &sc->pins[pinnumber].intr_rid, RF_ACTIVE, sc->pins[pinnumber].pin, intr_mode);
	if (sc->pins[pinnumber].intr_res == NULL) {
		device_printf(sc->dev, "cannot allocate interrupt resource\n");
		return (ENXIO);
	}

	err = bus_setup_intr(sc->pins[pinnumber].pin->dev, sc->pins[pinnumber].intr_res, INTR_TYPE_MISC | INTR_MPSAFE, NULL, gpiointr_interrupt_handler, sc, &sc->pins[pinnumber].intr_cookie);
	if (err != 0) {
		device_printf(sc->dev, "cannot set up interrupt\n");
		return (err);
	}

	sc->pins[pinnumber].configured = true;
	sc->active = true;

	device_printf(sc->dev, "interrupt on %s pin %d (%s)\n", device_get_nameunit(sc->pins[pinnumber].pin->dev), sc->pins[pinnumber].pin->pin, gpiointr_intr_mode_to_str(intr_mode));

	return (0);
}

static int
gpiointr_release_pin(struct gpiointr_softc *sc, int pinnumber)
{
	int err;

	err = 0;

	sc->pins[pinnumber].configured = false;

	/* Mark driver only as inactive when no pin is configured anymore */
	sc->active = false;
	for (int i = 0; i <= sc->npins; i++) {
		if (sc->pins[i].configured) {
			sc->active = true;
		}
	}

	/* Make pending IO syscalls return when no more pins are configured */
	if (!sc->active) {
		wakeup(sc);
		KNOTE_UNLOCKED(&sc->selinfo.si_note, 0);
		selwakeup(&sc->selinfo);
	}

	if (sc->pins[pinnumber].intr_cookie != NULL) {
		err = bus_teardown_intr(sc->pins[pinnumber].pin->dev, sc->pins[pinnumber].intr_res, sc->pins[pinnumber].intr_cookie);
		if (err != 0)
			device_printf(sc->dev, "cannot tear down interrupt\n");
		else
			sc->pins[pinnumber].intr_cookie = NULL;
	}

	if (sc->pins[pinnumber].intr_res != NULL) {
		err = bus_release_resource(sc->pins[pinnumber].pin->dev, SYS_RES_IRQ, sc->pins[pinnumber].intr_rid, sc->pins[pinnumber].intr_res);
		if (err != 0)
			device_printf(sc->dev, "cannot release interrupt resource\n");
		else
			sc->pins[pinnumber].intr_res = NULL;
	}

	return (0);
}

static int
gpiointr_probe(device_t dev)
{

	device_set_desc(dev, "GPIO interrupt userspace interface driver");
	return (BUS_PROBE_DEFAULT);
}

static int
gpiointr_attach(device_t dev)
{
	struct gpiointr_softc *sc;
	int err;
	const char *inst_name;
	struct make_dev_args dev_args;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->pdev = device_get_parent(dev);

	inst_name = device_get_nameunit(dev);

	mtx_init(&sc->mtx, inst_name, NULL, MTX_DEF);

	knlist_init_mtx(&sc->selinfo.si_note, &sc->mtx);

	err = GPIO_PIN_MAX(sc->pdev, &sc->npins);
	if (err != 0) {
		device_printf(dev, "cannot get number of pins\n");
		return (err);
	}

	sc->pins = malloc(sizeof(struct gpiointr_pin) * sc->npins, M_GPIOINTR, M_WAITOK | M_ZERO);

	for (int i = 0; i <= sc->npins; i++) {
		sc->pins[i].pin = malloc(sizeof(struct gpiobus_pin), M_GPIOINTR, M_WAITOK | M_ZERO);
		sc->pins[i].pin->pin = i;
		sc->pins[i].pin->dev = sc->pdev;
	}

	make_dev_args_init(&dev_args);

	dev_args.mda_devsw = &gpiointr_cdevsw;
	dev_args.mda_uid = UID_ROOT;
	dev_args.mda_gid = GID_WHEEL;
	dev_args.mda_mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;
	dev_args.mda_si_drv1 = sc;

	err = make_dev_s(&dev_args, &sc->cdev, "%s", inst_name);
	if (err != 0) {
		device_printf(dev, "cannot create device\n");
		return (err);
	}

	return (0);
}

static int
gpiointr_detach(device_t dev)
{
	struct gpiointr_softc *sc;

	sc = device_get_softc(dev);

	for (int i = 0; i <= sc->npins; i++)
		gpiointr_release_pin(sc, i);

	if (sc->cdev != NULL)
		destroy_dev(sc->cdev);

	for (int i = 0; i <= sc->npins; i++)
		free(sc->pins[i].pin, M_GPIOINTR);

	free(sc->pins, M_GPIOINTR);

	knlist_clear(&sc->selinfo.si_note, 0);
	knlist_destroy(&sc->selinfo.si_note);

	mtx_destroy(&sc->mtx);

	return (0);
}

static void
gpiointr_interrupt_handler(void *arg)
{
	struct gpiointr_softc *sc = arg;

	if (sc->active)
	{
		mtx_lock(&sc->mtx);

		wakeup(sc);

		sc->intr_toogle = !sc->intr_toogle;
		selwakeup(&sc->selinfo);

		KNOTE_LOCKED(&sc->selinfo.si_note, 1);

		sc->counter++;
		if (sc->counter == 0u)
			device_printf(sc->dev, "interrupt counter overflow\n");

		mtx_unlock(&sc->mtx);
	}
}

static void
gpiointr_cdevpriv_dtor(void *data)
{

	free(data, M_GPIOINTR);
}

static int
gpiointr_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct gpiointr_cdevpriv *priv;
	int err;

	priv = malloc(sizeof(*priv), M_GPIOINTR,  M_WAITOK | M_ZERO);

	err = devfs_set_cdevpriv(priv, gpiointr_cdevpriv_dtor);
	if (err != 0)
		gpiointr_cdevpriv_dtor(priv);
	priv->sc = dev->si_drv1;

	return (err);
}

static int
gpiointr_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{

	return (0);
}

static int
gpiointr_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct gpiointr_softc *sc = dev->si_drv1;
	int err;

	do {
		if (sc->active == true)
			err = tsleep(sc, PCATCH, "gpiointrwait", 20 * hz);
		else
			err = ENXIO;
	} while (err == EWOULDBLOCK);

	return (err);
}

static int
gpiointr_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
	struct gpiointr_softc *sc = dev->si_drv1;
	struct gpio_intr_config intr_config;
	unsigned int counter;
	int err;

	switch (cmd) {

	case GPIOINTRGETCONFIG:

		bcopy(data, &intr_config, sizeof(intr_config));

		if (intr_config.gp_pin < 0 || intr_config.gp_pin > sc->npins) {
			device_printf(sc->dev, "invalid pin %d\n", intr_config.gp_pin);
			return (EINVAL);
		}

		if (sc->pins[intr_config.gp_pin].configured == false) {
			device_printf(sc->dev, "pin %d is not configured\n", intr_config.gp_pin);
			return (ENXIO);
		}

		intr_config.gp_intr_flags = sc->pins[intr_config.gp_pin].pin->flags & GPIO_INTR_MASK;

		bcopy(&intr_config, data, sizeof(intr_config));

		break;

	case GPIOINTRSETCONFIG:

		bcopy(data, &intr_config, sizeof(intr_config));

		if (intr_config.gp_pin < 0 || intr_config.gp_pin > sc->npins) {
			device_printf(sc->dev, "invalid pin %d\n", intr_config.gp_pin);
			return (EINVAL);
		}

		if (sc->pins[intr_config.gp_pin].configured == true) {
			err = gpiointr_release_pin(sc, intr_config.gp_pin);
			if (err != 0)
				return (err);
		}

		/* Update pin */
		err = GPIO_PIN_GETFLAGS(sc->pins[intr_config.gp_pin].pin->dev, sc->pins[intr_config.gp_pin].pin->pin, &sc->pins[intr_config.gp_pin].pin->flags);
		if (err != 0) {
			device_printf(sc->dev, "cannot get flags of pin %d\n", intr_config.gp_pin);
			return (err);
		}
		sc->pins[intr_config.gp_pin].pin->flags &= ~GPIO_INTR_MASK;
		sc->pins[intr_config.gp_pin].pin->flags |= (intr_config.gp_intr_flags & GPIO_INTR_MASK);

		if ((sc->pins[intr_config.gp_pin].pin->flags & GPIO_INTR_MASK) != GPIO_INTR_NONE) {
			err = gpiointr_allocate_pin(sc, intr_config.gp_pin);
			if (err != 0) {
				gpiointr_release_pin(sc, intr_config.gp_pin);
				return (err);
			}
		} else {
			device_printf(sc->dev, "interrupt on %s pin %d removed\n", device_get_nameunit(sc->pins[intr_config.gp_pin].pin->dev), intr_config.gp_pin);
		}

		break;

	case GPIOINTRRESETCOUNTER:

		mtx_lock(&sc->mtx);
		sc->counter = 0u;
		mtx_unlock(&sc->mtx);

		break;

	case GPIOINTRGETCOUNTER:

		counter = sc->counter;
		bcopy(&counter, data, sizeof(counter));

		break;

	default:

		return (ENOTTY);

	}

	return (0);
}

static int
gpiointr_poll(struct cdev *dev, int events, struct thread *td)
{
	struct gpiointr_softc *sc = dev->si_drv1;
	struct gpiointr_cdevpriv *priv;
	int err;
	int revents;

	revents = 0;

	err = devfs_get_cdevpriv((void **)&priv);
	if (err != 0) {
		revents = POLLERR;
		return (revents);
	}

	if (sc->active == false) {
		revents = POLLHUP;
		priv->poll_en = false;
		return (revents);
	}

	mtx_lock(&sc->mtx);

	if (events & (POLLIN | POLLRDNORM)) {
		if (priv->poll_en && (sc->intr_toogle != priv->intr_toogle)) {
			revents |= POLLIN | POLLRDNORM;
			priv->poll_en = false;
		} else if (priv->poll_en && (sc->intr_toogle == priv->intr_toogle)) {
			selrecord(td, &sc->selinfo);
		} else {
			priv->poll_en = true;
			priv->intr_toogle = sc->intr_toogle;
			selrecord(td, &sc->selinfo);
		}
	}

	mtx_unlock(&sc->mtx);

	return (revents);
}

static int
gpiointr_kqfilter(struct cdev *dev, struct knote *kn)
{
        struct gpiointr_softc *sc = dev->si_drv1;
	struct gpiointr_cdevpriv *priv;
	struct knlist *knlist;
	int err;

	err = devfs_get_cdevpriv((void **)&priv);
	if (err != 0)
		return err;

	if (sc->active == false)
		return (ENXIO);

	switch(kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &gpiointr_read_filterops;
		kn->kn_hook = (void *)priv;
		break;
	default:
		return (EOPNOTSUPP);
	}

	knlist = &sc->selinfo.si_note;
	knlist_add(knlist, kn, 0);

	return (0);
}

static int
gpiointr_kqread(struct knote *kn, long hint)
{
	struct gpiointr_cdevpriv *priv = kn->kn_hook;
	struct gpiointr_softc *sc = priv->sc;

	if (sc->active == false) {
		kn->kn_flags |= EV_EOF;
		return (1);
	} else {
		if (hint != 0) {
			priv->kq_stale = true;
			return (1);
		}
		else if (priv->kq_stale == true) {
			priv->kq_stale = false;
			return (1);
		}
	}

	return (0);
}

static void
gpiointr_kqdetach(struct knote *kn)
{
	struct gpiointr_cdevpriv *priv = kn->kn_hook;
	struct gpiointr_softc *sc = priv->sc;
	struct knlist *knlist = &sc->selinfo.si_note;

	knlist_remove(knlist, kn, 0);
}

static device_method_t gpiointr_methods[] = {
	DEVMETHOD(device_probe,  gpiointr_probe),
	DEVMETHOD(device_attach, gpiointr_attach),
	DEVMETHOD(device_detach, gpiointr_detach),

	DEVMETHOD_END
};

static driver_t gpiointr_driver = {
	"gpiointr",
	gpiointr_methods,
	sizeof(struct gpiointr_softc)
};

static devclass_t gpiointr_devclass;

DRIVER_MODULE(gpiointr, gpio, gpiointr_driver, gpiointr_devclass, NULL, NULL);
MODULE_VERSION(gpiointr, 1);
MODULE_DEPEND(gpiointr, gpiobus, 1, 1, 1);
