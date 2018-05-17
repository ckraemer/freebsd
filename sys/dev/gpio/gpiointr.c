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
};

static int	gpiointr_probe(device_t);
static int	gpiointr_attach(device_t);
static int	gpiointr_detach(device_t);
static void	gpiointr_interrupt_handler(void*);
static int	gpiointr_open(struct cdev*, int, int, struct thread*);
static int	gpiointr_close(struct cdev*, int, int, struct thread*);
static int	gpiointr_read(struct cdev*, struct uio*, int);

static struct cdevsw gpiointr_cdevsw = {
	.d_version = D_VERSION,
	.d_open = gpiointr_open,
	.d_close = gpiointr_close,
	.d_read = gpiointr_read
};

static int
gpiointr_probe(device_t dev) {
	device_printf(dev, "probe\n");

	if (!ofw_bus_is_compatible(dev, "gpio-intr"))
		return (ENXIO);

	device_set_desc(dev, "GPIO interrupt userspace interface driver");
	return (BUS_PROBE_DEFAULT);
}

static int
gpiointr_attach(device_t dev) {
	struct gpiointr_softc *sc;
	phandle_t node;
	int err;
	int unit;
	struct make_dev_args dev_args;

	device_printf(dev, "attach\n");

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);
	if(node == -1)
	{
		device_printf(dev, "no node in fdt\n");
		return (ENXIO);
	}

	if(!OF_hasprop(node, "gpiopin"))
	{
		device_printf(dev, "no gpiopin property in fdt\n");
		return (ENXIO);
	}

	err = gpio_pin_get_by_ofw_propidx(dev, node, "gpiopin", 0, &sc->pin);
	if(err != 0)
	{
		device_printf(dev, "no valid gpiopin in fdt");
		return (err);
	}

	device_printf(dev, "gpiopin is pin number %d\n", sc->pin->pin);

	sc->intr_res = gpio_alloc_intr_resource(dev, &sc->intr_rid, RF_ACTIVE, sc->pin, GPIO_INTR_EDGE_FALLING);
	if(sc->intr_res == NULL)
	{
		device_printf(dev, "cannot allocate interrupt resource\n");
		return(ENXIO);
	}

	device_printf(dev, "interrupt resource allocated\n");

	err = bus_setup_intr(dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE, NULL, gpiointr_interrupt_handler, sc, &sc->intr_cookie);
	if(err != 0)
	{
		device_printf(dev, "cannot set up interrupt\n");
		return (err);
	}

	device_printf(dev, "interrupt resource set up\n");

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

	device_printf(dev, "created gpiointr%d character device\n", unit);

	return (0);
}

static int
gpiointr_detach(device_t dev) {
	device_printf(dev, "detach\n");
	return (0);
}

static void
gpiointr_interrupt_handler(void *arg)
{
	struct gpiointr_softc *sc = arg;

	device_printf(sc->dev, "interrupt handler executed!\n");
}

static int
gpiointr_open(struct cdev *dev, int oflags, int devtype, struct thread *td) {
	struct gpiointr_softc *sc = dev->si_drv1;
	device_printf(sc->dev, "open\n");
	return (0);
}

static int
gpiointr_close(struct cdev *dev, int fflag, int devtype, struct thread *td) {
	struct gpiointr_softc *sc = dev->si_drv1;
	device_printf(sc->dev, "close\n");
	return (0);
}

static int
gpiointr_read(struct cdev *dev, struct uio *uio, int ioflag) {
	struct gpiointr_softc *sc = dev->si_drv1;
	device_printf(sc->dev, "read\n");
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

DRIVER_MODULE(gpiointr, simplebus, gpiointr_driver, gpiointr_devclass, NULL, NULL);
MODULE_VERSION(gpiointr, 1);
MODULE_DEPEND(gpiointr, gpiobus, 1, 1, 1);
