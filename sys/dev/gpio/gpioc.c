/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2009 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/gpio.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/uio.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/module.h>

#include <dev/gpio/gpiobusvar.h>

#include "gpio_if.h"
#include "gpiobus_if.h"

#undef GPIOC_DEBUG
#ifdef GPIOC_DEBUG
#define dprintf printf
#else
#define dprintf(x, arg...)
#endif

struct gpioc_softc {
	device_t		sc_dev;		/* gpiocX dev */
	device_t		sc_pdev;	/* gpioX dev */
	struct cdev		*sc_ctl_dev;	/* controller device */
	int			sc_unit;
	int			sc_npins;
	struct gpioc_pin_intr	*sc_pin_intr;
};

struct gpioc_pin_intr {
	struct gpioc_softc				*sc;
	gpio_pin_t					pin;
	bool						config_locked;
	int						intr_rid;
	struct resource					*intr_res;
	void						*intr_cookie;
	struct mtx					mtx;
	SLIST_HEAD(gpioc_privs_list, gpioc_privs)	privs;
};

struct gpioc_cdevpriv {
	struct gpioc_softc			*sc;
	uint32_t				last_intr_pin;
	struct selinfo				selinfo;
	struct mtx				mtx;
	SLIST_HEAD(gpioc_pins_list, gpioc_pins)	pins;
};

struct gpioc_privs {
	struct gpioc_cdevpriv		*priv;
	SLIST_ENTRY(gpioc_privs)	next;
};

struct gpioc_pins {
	struct gpioc_pin_intr	*pin;
	SLIST_ENTRY(gpioc_pins)	next;
};

static MALLOC_DEFINE(M_GPIOC, "gpioc", "gpioc device data");

static int	gpioc_allocate_pin_intr(struct gpioc_pin_intr*, uint32_t);
static int	gpioc_release_pin_intr(struct gpioc_pin_intr*);
static int	gpioc_attach_priv_pin(struct gpioc_cdevpriv*,
		    struct gpioc_pin_intr*);
static int	gpioc_detach_priv_pin(struct gpioc_cdevpriv*,
		    struct gpioc_pin_intr*);
static bool	gpioc_intr_reconfig_allowed(struct gpioc_cdevpriv*,
		    struct gpioc_pin_intr *intr_conf);
static uint32_t	gpioc_get_intr_config(struct gpioc_softc*,
		    struct gpioc_cdevpriv*, uint32_t pin);
static int	gpioc_set_intr_config(struct gpioc_softc*,
		    struct gpioc_cdevpriv*, uint32_t, uint32_t);
static void	gpioc_interrupt_handler(void*);

static int gpioc_probe(device_t dev);
static int gpioc_attach(device_t dev);
static int gpioc_detach(device_t dev);

static void gpioc_cdevpriv_dtor(void*);

static d_open_t		gpioc_open;
static d_read_t		gpioc_read;
static d_ioctl_t	gpioc_ioctl;
static d_poll_t		gpioc_poll;
static d_kqfilter_t	gpioc_kqfilter;

static int		gpioc_kqread(struct knote*, long);
static void		gpioc_kqdetach(struct knote*);


static struct cdevsw gpioc_cdevsw = {
	.d_version	= D_VERSION,
	.d_open		= gpioc_open,
	.d_read		= gpioc_read,
	.d_ioctl	= gpioc_ioctl,
	.d_poll		= gpioc_poll,
	.d_kqfilter	= gpioc_kqfilter,
	.d_name		= "gpioc",
};

static struct filterops gpioc_read_filterops = {
	.f_isfd =	true,
	.f_attach =	NULL,
	.f_detach =	gpioc_kqdetach,
	.f_event =	gpioc_kqread,
	.f_touch =	NULL
};

static int
gpioc_allocate_pin_intr(struct gpioc_pin_intr *intr_conf, uint32_t flags)
{
	int err;

	intr_conf->config_locked = true;
	mtx_unlock(&intr_conf->mtx);

	intr_conf->intr_res = gpio_alloc_intr_resource(intr_conf->pin->dev,
	    &intr_conf->intr_rid, RF_ACTIVE, intr_conf->pin, flags);
	if (intr_conf->intr_res == NULL)
		return (ENXIO);

	err = bus_setup_intr(intr_conf->pin->dev, intr_conf->intr_res,
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, gpioc_interrupt_handler,
	    intr_conf, &intr_conf->intr_cookie);
	if (err != 0)
		return (err);

	intr_conf->pin->flags = flags;
	mtx_lock(&intr_conf->mtx);
	intr_conf->config_locked = false;
	wakeup(&intr_conf->config_locked);

	return (0);
}

static int
gpioc_release_pin_intr(struct gpioc_pin_intr *intr_conf)
{
	int err;

	intr_conf->config_locked = true;
	mtx_unlock(&intr_conf->mtx);

	if (intr_conf->intr_cookie != NULL) {
		err = bus_teardown_intr(intr_conf->pin->dev,
		    intr_conf->intr_res, intr_conf->intr_cookie);
		if (err != 0)
			return (err);
		else
			intr_conf->intr_cookie = NULL;
	}

	if (intr_conf->intr_res != NULL) {
		err = bus_release_resource(intr_conf->pin->dev, SYS_RES_IRQ,
		    intr_conf->intr_rid, intr_conf->intr_res);
		if (err != 0)
			return (err);
		else {
			intr_conf->intr_rid = 0;
			intr_conf->intr_res = NULL;
		}
	}

	intr_conf->pin->flags = 0;
	mtx_lock(&intr_conf->mtx);
	intr_conf->config_locked = false;
	wakeup(&intr_conf->config_locked);

	return (0);
}

static int
gpioc_attach_priv_pin(struct gpioc_cdevpriv *priv,
    struct gpioc_pin_intr *intr_conf)
{
	struct gpioc_privs	*priv_link;
	struct gpioc_pins	*pin_link;
	unsigned int		consistency_a, consistency_b;

	consistency_a = 0;
	consistency_b = 0;
	mtx_assert(&intr_conf->mtx, MA_OWNED);
	mtx_lock(&priv->mtx);
	SLIST_FOREACH(priv_link, &intr_conf->privs, next) {
		if (priv_link->priv == priv)
			consistency_a++;
	}
	KASSERT(consistency_a <= 1,
	    ("inconsistent links between pin config and cdevpriv"));
	SLIST_FOREACH(pin_link, &priv->pins, next) {
		if (pin_link->pin == intr_conf)
			consistency_b++;
	}
	KASSERT(consistency_a == consistency_b,
	    ("inconsistent links between pin config and cdevpriv"));
	if (consistency_a == 1 && consistency_b == 1) {
		mtx_unlock(&priv->mtx);
		mtx_unlock(&intr_conf->mtx);
		return (EEXIST);
	}
	priv_link = malloc(sizeof(struct gpioc_privs), M_GPIOC,
	    M_NOWAIT | M_ZERO);
	if (priv_link == NULL)
	{
		mtx_unlock(&priv->mtx);
		mtx_unlock(&intr_conf->mtx);
		return (ENOMEM);
	}
	pin_link = malloc(sizeof(struct gpioc_pins), M_GPIOC,
	    M_NOWAIT | M_ZERO);
	if (pin_link == NULL) {
		mtx_unlock(&priv->mtx);
		mtx_unlock(&intr_conf->mtx);
		return (ENOMEM);
	}
	priv_link->priv = priv;
	pin_link->pin = intr_conf;
	SLIST_INSERT_HEAD(&intr_conf->privs, priv_link, next);
	SLIST_INSERT_HEAD(&priv->pins, pin_link, next);
	mtx_unlock(&priv->mtx);

	return (0);
}

static int
gpioc_detach_priv_pin(struct gpioc_cdevpriv *priv,
    struct gpioc_pin_intr *intr_conf)
{
	struct gpioc_privs	*priv_link, *priv_link_temp;
	struct gpioc_pins	*pin_link, *pin_link_temp;
	unsigned int		consistency_a, consistency_b;

	consistency_a = 0;
	consistency_b = 0;
	mtx_assert(&intr_conf->mtx, MA_OWNED);
	mtx_lock(&priv->mtx);
	SLIST_FOREACH_SAFE(priv_link, &intr_conf->privs, next, priv_link_temp) {
		if (priv_link->priv == priv) {
			SLIST_REMOVE(&intr_conf->privs, priv_link, gpioc_privs,
			    next);
			free(priv_link, M_GPIOC);
			consistency_a++;
		}
	}
	KASSERT(consistency_a <= 1,
	    ("inconsistent links between pin config and cdevpriv"));
	SLIST_FOREACH_SAFE(pin_link, &priv->pins, next, pin_link_temp) {
		if (pin_link->pin == intr_conf) {
			SLIST_REMOVE(&priv->pins, pin_link, gpioc_pins, next);
			free(pin_link, M_GPIOC);
			consistency_b++;
		}
	}
	KASSERT(consistency_a == consistency_b,
	    ("inconsistent links between pin config and cdevpriv"));
	mtx_unlock(&priv->mtx);

	return (0);
}

static bool
gpioc_intr_reconfig_allowed(struct gpioc_cdevpriv *priv,
    struct gpioc_pin_intr *intr_conf)
{
	struct gpioc_privs	*priv_link;

	mtx_assert(&intr_conf->mtx, MA_OWNED);

	if (SLIST_EMPTY(&intr_conf->privs))
		return (true);

	SLIST_FOREACH(priv_link, &intr_conf->privs, next) {
		if (priv_link->priv != priv)
			return (false);
	}

	return (true);
}


static uint32_t
gpioc_get_intr_config(struct gpioc_softc *sc, struct gpioc_cdevpriv *priv,
    uint32_t pin)
{
	struct gpioc_pin_intr	*intr_conf = &sc->sc_pin_intr[pin];
	struct gpioc_privs	*priv_link;
	uint32_t		flags;

	flags = intr_conf->pin->flags;

	if (flags == 0)
		return (0);

	SLIST_FOREACH(priv_link, &intr_conf->privs, next) {
		if (priv_link->priv == priv) {
			flags |= GPIO_INTR_ATTACHED;
			break;
		}
	}

	return (flags);
}

static int
gpioc_set_intr_config(struct gpioc_softc *sc, struct gpioc_cdevpriv *priv,
    uint32_t pin, uint32_t flags)
{
	struct gpioc_pin_intr *intr_conf = &sc->sc_pin_intr[pin];
	int res;

	res = 0;
	if (intr_conf->pin->flags == 0 && flags == 0) {
		/* No interrupt configured and none requested: Do nothing. */
		return (0);
	}
	mtx_lock(&intr_conf->mtx);
	while (intr_conf->config_locked == true)
		mtx_sleep(&intr_conf->config_locked, &intr_conf->mtx, 0,
		    "gpicfg", 0);
	if (intr_conf->pin->flags == 0 && flags != 0) {
		/* No interrupt is configured, but one is requested: Allocate
		   and setup interrupt on the according pin. */
		res = gpioc_allocate_pin_intr(intr_conf, flags);
		if (res == 0)
			res = gpioc_attach_priv_pin(priv, intr_conf);
		if (res == EEXIST)
			res = 0;
	} else if (intr_conf->pin->flags == flags) {
		/* Same interrupt requested as already configured: Attach the
		   cdevpriv to the corresponding pin. */
		res = gpioc_attach_priv_pin(priv, intr_conf);
		if (res == EEXIST)
			res = 0;
	} else if (intr_conf->pin->flags != 0 && flags == 0) {
		/* Interrupt configured, but none requested: Teardown and
		   release the pin when no other cdevpriv is attached.
		   Otherwise just detach pin and cdevpriv from each other. */
		if (gpioc_intr_reconfig_allowed(priv, intr_conf)) {
			res = gpioc_release_pin_intr(intr_conf);
		}
		if (res == 0)
			res = gpioc_detach_priv_pin(priv, intr_conf);
	} else {
		/* Other flag requested than configured: Reconfigure when no
		   other cdevpriv is are attached to the pin. */
		if (!gpioc_intr_reconfig_allowed(priv, intr_conf))
			res = EBUSY;
		else {
			res = gpioc_release_pin_intr(intr_conf);
			if (res == 0)
				res = gpioc_allocate_pin_intr(intr_conf, flags);
			if (res == 0)
				res = gpioc_attach_priv_pin(priv, intr_conf);
			if (res == EEXIST)
				res = 0;
		}
	}
	mtx_unlock(&intr_conf->mtx);

	return (res);
}

static void
gpioc_interrupt_handler(void *arg)
{
	struct gpioc_pin_intr *intr_conf;
	struct gpioc_privs *privs;
	struct gpioc_softc *sc;

	intr_conf = arg;
	sc = intr_conf->sc;

	mtx_lock(&intr_conf->mtx);

	if (intr_conf->config_locked == true) {
		device_printf(sc->sc_dev, "Interrupt configuration in "
		    "progress. Discarding interrupt on pin %d.\n",
		    intr_conf->pin->pin);
		mtx_unlock(&intr_conf->mtx);
		return;
	}

	if (SLIST_EMPTY(&intr_conf->privs)) {
		device_printf(sc->sc_dev, "No file descriptor associated with "
		    "occurred interrupt on pin %d.\n", intr_conf->pin->pin);
		mtx_unlock(&intr_conf->mtx);
		return;
	}

	SLIST_FOREACH(privs, &intr_conf->privs, next) {
		mtx_lock(&privs->priv->mtx);
		if (privs->priv->last_intr_pin != -1)
			device_printf(sc->sc_dev, "Unhandled interrupt on pin "
			    "%d.\n", intr_conf->pin->pin);
		privs->priv->last_intr_pin = intr_conf->pin->pin;
		wakeup(privs->priv);
		selwakeup(&privs->priv->selinfo);
		KNOTE_LOCKED(&privs->priv->selinfo.si_note, 0);
		mtx_unlock(&privs->priv->mtx);
	}

	mtx_unlock(&intr_conf->mtx);
}

static int
gpioc_probe(device_t dev)
{
	device_set_desc(dev, "GPIO controller");
	return (0);
}

static int
gpioc_attach(device_t dev)
{
	int err;
	struct gpioc_softc *sc;
	struct make_dev_args devargs;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_pdev = device_get_parent(dev);
	sc->sc_unit = device_get_unit(dev);

	err = GPIO_PIN_MAX(sc->sc_pdev, &sc->sc_npins);
	if (err != 0)
		return (err);
	sc->sc_pin_intr = malloc(sizeof(struct gpioc_pin_intr) * sc->sc_npins,
	    M_GPIOC, M_WAITOK | M_ZERO);
	for (int i = 0; i <= sc->sc_npins; i++) {
		sc->sc_pin_intr[i].pin = malloc(sizeof(struct gpiobus_pin),
		    M_GPIOC, M_WAITOK | M_ZERO);
		sc->sc_pin_intr[i].sc = sc;
		sc->sc_pin_intr[i].pin->pin = i;
		sc->sc_pin_intr[i].pin->dev = sc->sc_pdev;
		mtx_init(&sc->sc_pin_intr[i].mtx, "gpioc pin", NULL, MTX_DEF);
		SLIST_INIT(&sc->sc_pin_intr[i].privs);
	}

	make_dev_args_init(&devargs);
	devargs.mda_devsw = &gpioc_cdevsw;
	devargs.mda_uid = UID_ROOT;
	devargs.mda_gid = GID_WHEEL;
	devargs.mda_mode = 0600;
	devargs.mda_si_drv1 = sc;
	err = make_dev_s(&devargs, &sc->sc_ctl_dev, "gpioc%d", sc->sc_unit);
	if (err != 0) {
		printf("Failed to create gpioc%d", sc->sc_unit);
		return (ENXIO);
	}

	return (0);
}

static int
gpioc_detach(device_t dev)
{
	struct gpioc_softc *sc = device_get_softc(dev);
	int err;

	if (sc->sc_ctl_dev)
		destroy_dev(sc->sc_ctl_dev);

	for (int i = 0; i <= sc->sc_npins; i++) {
		mtx_destroy(&sc->sc_pin_intr[i].mtx);
		free(&sc->sc_pin_intr[i].pin, M_GPIOC);
	}
	free(sc->sc_pin_intr, M_GPIOC);

	if ((err = bus_generic_detach(dev)) != 0)
		return (err);

	return (0);
}

static void
gpioc_cdevpriv_dtor(void *data)
{
	struct gpioc_cdevpriv	*priv;
	struct gpioc_privs	*priv_link, *priv_link_temp;
	struct gpioc_pins	*pin_link, *pin_link_temp;
	unsigned int		consistency;

	priv = data;

	mtx_lock(&priv->mtx);
	SLIST_FOREACH_SAFE(pin_link, &priv->pins, next, pin_link_temp) {
		consistency = 0;
		mtx_lock(&pin_link->pin->mtx);
		while (pin_link->pin->config_locked == true)
			mtx_sleep(&pin_link->pin->config_locked,
			    &pin_link->pin->mtx, 0, "gpicfg", 0);
		SLIST_FOREACH_SAFE(priv_link, &pin_link->pin->privs, next,
		    priv_link_temp) {
			if (priv_link->priv == priv) {
				SLIST_REMOVE(&pin_link->pin->privs, priv_link,
				    gpioc_privs, next);
				free(priv_link, M_GPIOC);
				consistency++;
			}
		}
		KASSERT(consistency == 1,
		    ("inconsistent links between pin config and cdevpriv"));
		if (gpioc_intr_reconfig_allowed(priv, pin_link->pin)) {
			gpioc_release_pin_intr(pin_link->pin);
		}
		mtx_unlock(&pin_link->pin->mtx);
		SLIST_REMOVE(&priv->pins, pin_link, gpioc_pins, next);
		free(pin_link, M_GPIOC);
	}
	mtx_unlock(&priv->mtx);

	wakeup(&priv);
	knlist_clear(&priv->selinfo.si_note, 0);
	seldrain(&priv->selinfo);
	knlist_destroy(&priv->selinfo.si_note);

	mtx_destroy(&priv->mtx);
	free(data, M_GPIOC);
}

static int
gpioc_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct gpioc_cdevpriv *priv;
	int err;

	priv = malloc(sizeof(*priv), M_GPIOC, M_WAITOK | M_ZERO);
	err = devfs_set_cdevpriv(priv, gpioc_cdevpriv_dtor);
	if (err != 0) {
		gpioc_cdevpriv_dtor(priv);
		return (err);
	}
	priv->sc = dev->si_drv1;
	priv->last_intr_pin = -1;
	mtx_init(&priv->mtx, "gpioc priv", NULL, MTX_DEF);
	knlist_init_mtx(&priv->selinfo.si_note, &priv->mtx);

	return (0);
}

static int
gpioc_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct gpioc_cdevpriv *priv;
	uint32_t last_intr_pin;
	int err;

	if (uio->uio_resid < sizeof(priv->last_intr_pin))
		return EINVAL;

	err = devfs_get_cdevpriv((void **)&priv);
	if (err != 0)
		return err;

	mtx_lock(&priv->mtx);
	do {
		if (!SLIST_EMPTY(&priv->pins))
			err = mtx_sleep(priv, &priv->mtx, PCATCH, "gpintr", 0);
		else
			err = ENXIO;
	} while (err == EWOULDBLOCK);

	if (err == 0 && priv->last_intr_pin != -1)
	{
		last_intr_pin = priv->last_intr_pin;
		priv->last_intr_pin = -1;
		mtx_unlock(&priv->mtx);
		err = uiomove(&last_intr_pin, sizeof(last_intr_pin), uio);
	}
	else {
		mtx_unlock(&priv->mtx);
	}

	return (err);
}

static int 
gpioc_ioctl(struct cdev *cdev, u_long cmd, caddr_t arg, int fflag, 
    struct thread *td)
{
	device_t bus;
	int max_pin, res;
	struct gpioc_softc *sc = cdev->si_drv1;
	struct gpioc_cdevpriv *priv;
	struct gpio_pin pin;
	struct gpio_req req;
	struct gpio_access_32 *a32;
	struct gpio_config_32 *c32;
	uint32_t caps;

	bus = GPIO_GET_BUS(sc->sc_pdev);
	if (bus == NULL)
		return (EINVAL);
	switch (cmd) {
		case GPIOMAXPIN:
			max_pin = -1;
			res = GPIO_PIN_MAX(sc->sc_pdev, &max_pin);
			bcopy(&max_pin, arg, sizeof(max_pin));
			break;
		case GPIOGETCONFIG:
			bcopy(arg, &pin, sizeof(pin));
			dprintf("get config pin %d\n", pin.gp_pin);
			res = GPIO_PIN_GETFLAGS(sc->sc_pdev, pin.gp_pin,
			    &pin.gp_flags);
			/* Fail early */
			if (res)
				break;
			res = devfs_get_cdevpriv((void **)&priv);
			if (res)
				break;
			pin.gp_flags |= gpioc_get_intr_config(sc, priv,
			    pin.gp_pin);
			GPIO_PIN_GETCAPS(sc->sc_pdev, pin.gp_pin, &pin.gp_caps);
			GPIOBUS_PIN_GETNAME(bus, pin.gp_pin, pin.gp_name);
			bcopy(&pin, arg, sizeof(pin));
			break;
		case GPIOSETCONFIG:
			bcopy(arg, &pin, sizeof(pin));
			dprintf("set config pin %d\n", pin.gp_pin);
			res = devfs_get_cdevpriv((void **)&priv);
			if (res == 0)
				res = GPIO_PIN_GETCAPS(sc->sc_pdev, pin.gp_pin,
				    &caps);
			if (res == 0)
				res = gpio_check_flags(caps, pin.gp_flags);
			if (res == 0)
				res = GPIO_PIN_SETFLAGS(sc->sc_pdev, pin.gp_pin,
				    (pin.gp_flags & ~GPIO_INTR_MASK));
			if (res == 0)
				res = gpioc_set_intr_config(sc, priv,
				    pin.gp_pin,
				    (pin.gp_flags & GPIO_INTR_MASK));
			break;
		case GPIOGET:
			bcopy(arg, &req, sizeof(req));
			res = GPIO_PIN_GET(sc->sc_pdev, req.gp_pin,
			    &req.gp_value);
			dprintf("read pin %d -> %d\n", 
			    req.gp_pin, req.gp_value);
			bcopy(&req, arg, sizeof(req));
			break;
		case GPIOSET:
			bcopy(arg, &req, sizeof(req));
			res = GPIO_PIN_SET(sc->sc_pdev, req.gp_pin, 
			    req.gp_value);
			dprintf("write pin %d -> %d\n", 
			    req.gp_pin, req.gp_value);
			break;
		case GPIOTOGGLE:
			bcopy(arg, &req, sizeof(req));
			dprintf("toggle pin %d\n", 
			    req.gp_pin);
			res = GPIO_PIN_TOGGLE(sc->sc_pdev, req.gp_pin);
			break;
		case GPIOSETNAME:
			bcopy(arg, &pin, sizeof(pin));
			dprintf("set name on pin %d\n", pin.gp_pin);
			res = GPIOBUS_PIN_SETNAME(bus, pin.gp_pin,
			    pin.gp_name);
			break;
		case GPIOACCESS32:
			a32 = (struct gpio_access_32 *)arg;
			res = GPIO_PIN_ACCESS_32(sc->sc_pdev, a32->first_pin,
			    a32->clear_pins, a32->orig_pins, &a32->orig_pins);
			break;
		case GPIOCONFIG32:
			c32 = (struct gpio_config_32 *)arg;
			res = GPIO_PIN_CONFIG_32(sc->sc_pdev, c32->first_pin,
			    c32->num_pins, c32->pin_flags);
			break;
		default:
			return (ENOTTY);
			break;
	}

	return (res);
}

static int
gpioc_poll(struct cdev *dev, int events, struct thread *td)
{
	struct gpioc_cdevpriv *priv;
	int err;
	int revents;

	revents = 0;

	err = devfs_get_cdevpriv((void **)&priv);
	if (err != 0) {
		revents = POLLERR;
		return (revents);
	}

	if (SLIST_EMPTY(&priv->pins)) {
		revents = POLLHUP;
		return (revents);
	}

	if (events & (POLLIN | POLLRDNORM)) {
		if (priv->last_intr_pin != -1)
			revents |= events & (POLLIN | POLLRDNORM);
		else
			selrecord(td, &priv->selinfo);
	}

	return (revents);
}

static int
gpioc_kqfilter(struct cdev *dev, struct knote *kn)
{
	struct gpioc_cdevpriv *priv;
	struct knlist *knlist;
	int err;

	err = devfs_get_cdevpriv((void **)&priv);
	if (err != 0)
		return err;

	if (SLIST_EMPTY(&priv->pins))
		return (ENXIO);

	switch(kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &gpioc_read_filterops;
		kn->kn_hook = (void *)priv;
		break;
	default:
		return (EOPNOTSUPP);
	}

	knlist = &priv->selinfo.si_note;
	knlist_add(knlist, kn, 0);

	return (0);
}

static int
gpioc_kqread(struct knote *kn, long hint)
{
	struct gpioc_cdevpriv *priv = kn->kn_hook;

	if (SLIST_EMPTY(&priv->pins)) {
		kn->kn_flags |= EV_EOF;
		return (1);
	} else {
		if (priv->last_intr_pin != -1) {
			kn->kn_data = sizeof(priv->last_intr_pin);
			return (1);
		}
	}
	return (0);
}

static void
gpioc_kqdetach(struct knote *kn)
{
	struct gpioc_cdevpriv *priv = kn->kn_hook;
	struct knlist *knlist = &priv->selinfo.si_note;

	knlist_remove(knlist, kn, 0);
}

static device_method_t gpioc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		gpioc_probe),
	DEVMETHOD(device_attach,	gpioc_attach),
	DEVMETHOD(device_detach,	gpioc_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),

	DEVMETHOD_END
};

driver_t gpioc_driver = {
	"gpioc",
	gpioc_methods,
	sizeof(struct gpioc_softc)
};

devclass_t	gpioc_devclass;

DRIVER_MODULE(gpioc, gpio, gpioc_driver, gpioc_devclass, 0, 0);
MODULE_VERSION(gpioc, 1);
