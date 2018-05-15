#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

struct gpiointr_softc {
};

static int gpiointr_probe(device_t);
static int gpiointr_attach(device_t);
static int gpiointr_detach(device_t);

static int
gpiointr_probe(device_t dev) {
	device_printf(dev, "probe\n");
	device_set_desc(dev, "GPIO interrupt userspace interface driver");
	return (0);
}

static int
gpiointr_attach(device_t dev) {
	device_printf(dev, "attach\n");
	return (0);
}

static int
gpiointr_detach(device_t dev) {
	device_printf(dev, "detach\n");
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
