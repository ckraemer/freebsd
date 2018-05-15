.PATH:	${SRCTOP}/sys/dev/gpio/

KMOD=	gpiointr
SRCS=	gpiointr.c
SRCS+=	device_if.h bus_if.h gpio_if.h gpiobus_if.h opt_platform.h ofw_bus_if.h

CFLAGS+= -I. -I${SRCTOP}/sys/dev/gpio/

.include <bsd.kmod.mk>
