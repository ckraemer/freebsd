#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/gpio.h>
#include <sys/conf.h>
#include <sys/stat.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>

#include <dev/gpio/gpiobusvar.h>

struct gpiointr_softc {
	device_t	dev;
	gpio_pin_t	pin;
	int		intr_rid;
	struct resource	*intr_res;
	void		*intr_cookie;
	struct cdev     *cdev;
	bool		active;
};

static int	gpiointr_allocate_pin(struct gpiointr_softc*);
static int	gpiointr_release_pin(struct gpiointr_softc*);
static int	gpiointr_probe(device_t);
static int	gpiointr_attach(device_t);
static int	gpiointr_detach(device_t);
static void	gpiointr_interrupt_handler(void*);
static int	gpiointr_open(struct cdev*, int, int, struct thread*);
static int	gpiointr_close(struct cdev*, int, int, struct thread*);
static int	gpiointr_read(struct cdev*, struct uio*, int);
static int	gpiointr_ioctl(struct cdev*, u_long, caddr_t, int, struct thread*);

static struct cdevsw gpiointr_cdevsw = {
	.d_version = D_VERSION,
	.d_open = gpiointr_open,
	.d_close = gpiointr_close,
	.d_read = gpiointr_read,
	.d_ioctl = gpiointr_ioctl
};

static int
gpiointr_allocate_pin(struct gpiointr_softc *sc)
{
	device_t busdev;
	int err;

	busdev = GPIO_GET_BUS(sc->pin->dev);
	if (busdev == NULL)
		return (ENODEV);

	err = gpiobus_acquire_pin(busdev, sc->pin->pin);
	if (err != 0)
		return(EBUSY);

	sc->intr_res = gpio_alloc_intr_resource(sc->pin->dev, &sc->intr_rid, RF_ACTIVE, sc->pin, GPIO_INTR_EDGE_FALLING);
	if (sc->intr_res == NULL) {
		device_printf(sc->dev, "cannot allocate interrupt resource\n");
		return(ENXIO);
	}

	err = bus_setup_intr(sc->pin->dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE, NULL, gpiointr_interrupt_handler, sc, &sc->intr_cookie);
	if (err != 0) {
		device_printf(sc->dev, "cannot set up interrupt\n");
		return (err);
	}

	sc->active = true;

	device_printf(sc->dev, "interrupt on %s pin %d (falling edge)\n", device_get_nameunit(sc->pin->dev), sc->pin->pin);

	return (0);
}

static int
gpiointr_release_pin(struct gpiointr_softc *sc)
{
	device_t busdev;

	sc->active = false;
	wakeup(sc);

	bus_teardown_intr(sc->pin->dev, sc->intr_res, sc->intr_cookie);
	bus_release_resource(sc->pin->dev, SYS_RES_IRQ, sc->intr_rid, sc->intr_res);

	busdev = GPIO_GET_BUS(sc->pin->dev);
	if (busdev == NULL)
		return (ENODEV);

	gpiobus_release_pin(busdev, sc->pin->pin);

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
	int unit;
	struct make_dev_args dev_args;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->pin = malloc(sizeof(struct gpiobus_pin), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->pin->dev = device_get_parent(dev);

	unit = device_get_unit(dev);

	make_dev_args_init(&dev_args);

	dev_args.mda_devsw = &gpiointr_cdevsw;
	dev_args.mda_uid = UID_ROOT;
	dev_args.mda_gid = GID_WHEEL;
	dev_args.mda_mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP;
	dev_args.mda_si_drv1 = sc;

	err = make_dev_s(&dev_args, &sc->cdev, "gpiointr%d", unit);
	if (err != 0) {
		device_printf(dev, "cannot create gpiointr dev\n");
		return (err);
	}

	return (0);
}

static int
gpiointr_detach(device_t dev)
{
	struct gpiointr_softc *sc;

	sc = device_get_softc(dev);

	if (sc->active == true)
		gpiointr_release_pin(sc);
	destroy_dev(sc->cdev);

	return (0);
}

static void
gpiointr_interrupt_handler(void *arg)
{
	struct gpiointr_softc *sc = arg;

	wakeup(sc);
}

static int
gpiointr_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{

	return (0);
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
		err = tsleep(sc, PCATCH, "gpiointrwait", 20 * hz);
		if (sc->active == false) {
			err = EINTR;
		}
	} while (err == EWOULDBLOCK);

	return (err);
}

static int
gpiointr_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
	struct gpiointr_softc *sc = dev->si_drv1;
	struct gpio_intr_config intr_config;
	int err;

	switch (cmd) {
	case GPIOINTRCONFIG:
		bcopy(data, &intr_config, sizeof(intr_config));

		if (sc->active == true) {
			err = gpiointr_release_pin(sc);
			if (err != 0)
				return (err);
		}

		/* Update pin */
		sc->pin->pin = intr_config.gp_pin;
		err = GPIO_PIN_GETFLAGS(sc->pin->dev, sc->pin->pin, &sc->pin->flags);
		if (err != 0) {
			device_printf(sc->dev, "cannot get flags of pin %d\n", sc->pin->pin);
			return (err);
		}

		err = gpiointr_allocate_pin(sc);
		if (err != 0)
			return (err);
		break;
	default:
		return (ENOTTY);
	}

	return (0);
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
