#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/can/netlink.h>
#include <linux/types.h>

/* imported from kernel */

/**
 * abs - return absolute value of an argument
 * @x: the value.  If it is unsigned type, it is converted to signed type first.
 *     char is treated as if it was signed (regardless of whether it really is)
 *     but the macro's return type is preserved as char.
 *
 * Return: an absolute value of x.
 */
#define abs(x)	__abs_choose_expr(x, long long,				\
		__abs_choose_expr(x, long,				\
		__abs_choose_expr(x, int,				\
		__abs_choose_expr(x, short,				\
		__abs_choose_expr(x, char,				\
		__builtin_choose_expr(					\
			__builtin_types_compatible_p(typeof(x), char),	\
			(char)({ signed char __x = (x); __x<0?-__x:__x; }), \
			((void)0)))))))

#define __abs_choose_expr(x, type, other) __builtin_choose_expr(	\
	__builtin_types_compatible_p(typeof(x),   signed type) ||	\
	__builtin_types_compatible_p(typeof(x), unsigned type),		\
	({ signed type __x = (x); __x < 0 ? -__x : __x; }), other)

/*
 * min()/max()/clamp() macros that also do
 * strict type-checking.. See the
 * "unnecessary" pointer comparison.
 */
#define min(x, y) ({				\
	typeof(x) _min1 = (x);			\
	typeof(y) _min2 = (y);			\
	(void) (&_min1 == &_min2);		\
	_min1 < _min2 ? _min1 : _min2; })

#define max(x, y) ({				\
	typeof(x) _max1 = (x);			\
	typeof(y) _max2 = (y);			\
	(void) (&_max1 == &_max2);		\
	_max1 > _max2 ? _max1 : _max2; })

/**
 * clamp - return a value clamped to a given range with strict typechecking
 * @val: current value
 * @lo: lowest allowable value
 * @hi: highest allowable value
 *
 * This macro does strict typechecking of lo/hi to make sure they are of the
 * same type as val.  See the unnecessary pointer comparisons.
 */
#define clamp(val, lo, hi) min((typeof(val))max(val, lo), hi)

# define do_div(n,base) ({					\
	uint32_t __base = (base);				\
	uint32_t __rem;						\
	__rem = ((uint64_t)(n)) % __base;			\
	(n) = ((uint64_t)(n)) / __base;				\
	__rem;							\
 })

/* */

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* we don't want to see these prints */
#define netdev_err(dev, format, arg...) do { } while (0)
#define netdev_warn(dev, format, arg...) do { } while (0)

/* define in-kernel-types */
typedef __u64 u64;
typedef __u32 u32;

struct calc_ref_clk {
	__u32 clk;	/* CAN system clock frequency in Hz */
	char *name;
};

struct calc_bittiming_const {
	struct can_bittiming_const bittiming_const;

	const struct calc_ref_clk ref_clk[16];
	void (*printf_btr)(struct can_bittiming *bt, bool hdr);
};

/*
 * minimal structs, just enough to be source level compatible
 */
struct can_priv {
	struct can_clock clock;
};

struct net_device {
	struct can_priv	priv;
};

static inline void *netdev_priv(const struct net_device *dev)
{
	return (void *)&dev->priv;
}

#define CAN_CALC_MAX_ERROR 50 /* in one-tenth of a percent */
#define CAN_CALC_SYNC_SEG 1



static void printf_cfuc(struct can_bittiming *bt, bool hdr){}

static struct calc_bittiming_const can_calc_consts[] = {
	{
		.bittiming_const = {
			.name = "cfuc",
			.tseg1_min = 1,
			.tseg1_max = 16,
			.tseg2_min = 1,
			.tseg2_max = 8,
			.sjw_max = 4,
			.brp_min = 1,
			.brp_max = 64,
			.brp_inc = 1,
		},
		.ref_clk = {
			{ .clk = 144000000, },
		},
		.printf_btr = printf_cfuc,
	}
};

static long common_bitrates[] = {
	1000000,
	800000,
	500000,
	250000,
	125000,
	100000,
	50000,
	20000,
	10000,
};



/*
 * Bit-timing calculation derived from:
 *
 * Code based on LinCAN sources and H8S2638 project
 * Copyright 2004-2006 Pavel Pisa - DCE FELK CVUT cz
 * Copyright 2005      Stanislav Marek
 * email: pisa@cmp.felk.cvut.cz
 *
 * Calculates proper bit-timing parameters for a specified bit-rate
 * and sample-point, which can then be used to set the bit-timing
 * registers of the CAN controller. You can find more information
 * in the header file linux/can/netlink.h.
 */
static int can_update_spt(const struct can_bittiming_const *btc,
			  unsigned int spt_nominal, unsigned int tseg,
			  unsigned int *tseg1_ptr, unsigned int *tseg2_ptr,
			  unsigned int *spt_error_ptr)
{
	unsigned int spt_error, best_spt_error = UINT_MAX;
	unsigned int spt, best_spt = 0;
	unsigned int tseg1, tseg2;
	int i;

	for (i = 0; i <= 1; i++) {
		tseg2 = tseg + CAN_CALC_SYNC_SEG - (spt_nominal * (tseg + CAN_CALC_SYNC_SEG)) / 1000 - i;
		tseg2 = clamp(tseg2, btc->tseg2_min, btc->tseg2_max);
		tseg1 = tseg - tseg2;
		if (tseg1 > btc->tseg1_max) {
			tseg1 = btc->tseg1_max;
			tseg2 = tseg - tseg1;
		}

		spt = 1000 * (tseg + CAN_CALC_SYNC_SEG - tseg2) / (tseg + CAN_CALC_SYNC_SEG);
		spt_error = abs(spt_nominal - spt);

		if ((spt <= spt_nominal) && (spt_error < best_spt_error)) {
			best_spt = spt;
			best_spt_error = spt_error;
			*tseg1_ptr = tseg1;
			*tseg2_ptr = tseg2;
		}
	}

	if (spt_error_ptr)
		*spt_error_ptr = best_spt_error;

	return best_spt;
}

static int can_calc_bittiming(struct net_device *dev, struct can_bittiming *bt,
			      const struct can_bittiming_const *btc)
{
	struct can_priv *priv = netdev_priv(dev);
	unsigned int rate;		/* current bitrate */
	unsigned int rate_error;	/* difference between current and nominal value */
	unsigned int best_rate_error = UINT_MAX;
	unsigned int spt_error;		/* difference between current and nominal value */
	unsigned int best_spt_error = UINT_MAX;
	unsigned int spt_nominal;	/* nominal sample point */
	unsigned int best_tseg = 0;	/* current best value for tseg */
	unsigned int best_brp = 0;	/* current best value for brp */
	unsigned int brp, tsegall, tseg, tseg1 = 0, tseg2 = 0;
	u64 v64;

	/* Use CiA recommended sample points */
	if (bt->sample_point) {
		spt_nominal = bt->sample_point;
	} else {
		if (bt->bitrate > 800000)
			spt_nominal = 750;
		else if (bt->bitrate > 500000)
			spt_nominal = 800;
		else
			spt_nominal = 875;
	}

	/* tseg even = round down, odd = round up */
	for (tseg = (btc->tseg1_max + btc->tseg2_max) * 2 + 1;
	     tseg >= (btc->tseg1_min + btc->tseg2_min) * 2; tseg--) {
		tsegall = CAN_CALC_SYNC_SEG + tseg / 2;

		/* Compute all possible tseg choices (tseg=tseg1+tseg2) */
		brp = priv->clock.freq / (tsegall * bt->bitrate) + tseg % 2;

		/* choose brp step which is possible in system */
		brp = (brp / btc->brp_inc) * btc->brp_inc;
		if ((brp < btc->brp_min) || (brp > btc->brp_max))
			continue;

		rate = priv->clock.freq / (brp * tsegall);
		rate_error = abs(bt->bitrate - rate);

		/* tseg brp biterror */
		if (rate_error > best_rate_error)
			continue;

		/* reset sample point error if we have a better bitrate */
		if (rate_error < best_rate_error)
			best_spt_error = UINT_MAX;

		can_update_spt(btc, spt_nominal, tseg / 2, &tseg1, &tseg2, &spt_error);
		if (spt_error > best_spt_error)
			continue;

		best_spt_error = spt_error;
		best_rate_error = rate_error;
		best_tseg = tseg / 2;
		best_brp = brp;

		if (rate_error == 0 && spt_error == 0)
			break;
	}

	if (best_rate_error) {
		/* Error in one-tenth of a percent */
		rate_error = (best_rate_error * 1000) / bt->bitrate;
		if (rate_error > CAN_CALC_MAX_ERROR) {
			netdev_err(dev,
				   "bitrate error %ld.%ld%% too high\n",
				   rate_error / 10, rate_error % 10);
			return -EDOM;
		}
		netdev_warn(dev, "bitrate error %ld.%ld%%\n",
			    rate_error / 10, rate_error % 10);
	}

	/* real sample point */
	bt->sample_point = can_update_spt(btc, spt_nominal, best_tseg,
					  &tseg1, &tseg2, NULL);

	v64 = (u64)best_brp * 1000 * 1000 * 1000;
	do_div(v64, priv->clock.freq);
	bt->tq = (u32)v64;
	bt->prop_seg = tseg1 / 2;
	bt->phase_seg1 = tseg1 - bt->prop_seg;
	bt->phase_seg2 = tseg2;

	/* check for sjw user settings */
	if (!bt->sjw || !btc->sjw_max) {
		bt->sjw = 1;
	} else {
		/* bt->sjw is at least 1 -> sanitize upper bound to sjw_max */
		if (bt->sjw > btc->sjw_max)
			bt->sjw = btc->sjw_max;
		/* bt->sjw must not be higher than tseg2 */
		if (tseg2 < bt->sjw)
			bt->sjw = tseg2;
	}

	bt->brp = best_brp;

	/* real bit-rate */
	bt->bitrate = priv->clock.freq / (bt->brp * (CAN_CALC_SYNC_SEG + tseg1 + tseg2));

	return 0;
}

static __u32 get_cia_sample_point(__u32 bitrate)
{
	__u32 sampl_pt;

	if (bitrate > 800000)
		sampl_pt = 750;
	else if (bitrate > 500000)
		sampl_pt = 800;
	else
		sampl_pt = 875;

	return sampl_pt;
}

int cfuc_cal_baudrate (unsigned int bitrate_nominal,struct can_bittiming* bt){
	struct calc_ref_clk ref_clk = {
		.name = "uccb",
        .clk = 144000000
	};
    struct net_device dev = {
		.priv.clock.freq = ref_clk.clk,
	};
	
    bt->bitrate = bitrate_nominal;

    if (can_calc_bittiming(&dev, bt, &can_calc_consts->bittiming_const)) {
		printf("%7d ***bitrate not possible***\n", bitrate_nominal);
		return -1;
	}
    return 0;
}