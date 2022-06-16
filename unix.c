static void
futex_put(struct futex *f, struct waiting_proc *wp)
{
	LIN_SDT_PROBE2(futex, futex_put, entry, f, wp);

	if (wp != NULL) {
		if ((wp->wp_flags & FUTEX_WP_REMOVED) == 0)
			TAILQ_REMOVE(&f->f_waiting_proc, wp, wp_list);
		free(wp, M_FUTEX_WP);
	}

	FUTEXES_LOCK;
	if (--f->f_refcount == 0) {
		LIST_REMOVE(f, f_list);
		FUTEXES_UNLOCK;
		if (FUTEX_LOCKED(f))
			futex_unlock(f);

		LIN_SDT_PROBE3(futex, futex_put, destroy, f->f_uaddr,
		    f->f_refcount, f->f_key.shared);
		LINUX_CTR3(sys_futex, "futex_put destroy uaddr %p ref %d "
		    "shared %d", f->f_uaddr, f->f_refcount, f->f_key.shared);
		umtx_key_release(&f->f_key);
		FUTEX_DESTROY(f);
		free(f, M_FUTEX);

		LIN_SDT_PROBE0(futex, futex_put, return);
		return;
	}

	LIN_SDT_PROBE3(futex, futex_put, unlock, f->f_uaddr, f->f_refcount,
	    f->f_key.shared);
	LINUX_CTR3(sys_futex, "futex_put uaddr %p ref %d shared %d",
	    f->f_uaddr, f->f_refcount, f->f_key.shared);
	if (FUTEX_LOCKED(f))
		futex_unlock(f);
	FUTEXES_UNLOCK;

	LIN_SDT_PROBE0(futex, futex_put, return);
}
//====

#define SDT_PROBE(prov, mod, func, name, arg0, arg1, arg2, arg3, arg4)	do {	\
	if (SDT_PROBES_ENABLED()) {						\
		if (__predict_false(sdt_##prov##_##mod##_##func##_##name->id))	\
		(*sdt_probe_func)(sdt_##prov##_##mod##_##func##_##name->id,	\
		    (uintptr_t) arg0, (uintptr_t) arg1, (uintptr_t) arg2,	\
		    (uintptr_t) arg3, (uintptr_t) arg4);			\
	} \
} while (0)


//===
#define FUTEXES_LOCK		do { \
				    mtx_lock(&futex_mtx); \
				    LIN_SDT_PROBE1(locks, futex_mtx, \
					locked, &futex_mtx); \
				} while (0)

struct mtx futex_mtx;			/* protects the futex list */


typedef struct
{
  /* Boolean attributes.  */
  unsigned int bool;
  /* Non-boolean integer attributes.  */
  CGEN_ATTR_VALUE_TYPE nonbool[1];
} CGEN_ATTR;

typedef char boolean;
# define false 0
# define true 1

# undef MAX
# undef MIN
# define MAX(a, b) ((a) > (b) ? (a) : (b))
# define MIN(a, b) ((a) < (b) ? (a) : (b))


/* (Re)Allocate N items of type T using malloc, or fail.  */
# define TALLOC(n, t) ((t *) malloc ((n) * sizeof (t)))
# define RETALLOC(addr, n, t) ((addr) = (t *) realloc (addr, (n) * sizeof (t)))
# define RETALLOC_IF(addr, n, t) \
  if (addr) RETALLOC((addr), (n), t); else (addr) = TALLOC ((n), t)
# define REGEX_TALLOC(n, t) ((t *) REGEX_ALLOCATE ((n) * sizeof (t)))

# define BYTEWIDTH 8 /* In bits.  */

# define STREQ(s1, s2) ((strcmp (s1, s2) == 0))


union cvmx_pko_reg_min_pkt {
	uint64_t u64;
	struct cvmx_pko_reg_min_pkt_s {
#ifdef __BIG_ENDIAN_BITFIELD
	uint64_t size7                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size6                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size5                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size4                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size3                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size2                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size1                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
	uint64_t size0                        : 8;  /**< Minimum packet size-1 in bytes                                NS */
#else
	uint64_t size0                        : 8;
	uint64_t size1                        : 8;
	uint64_t size2                        : 8;
	uint64_t size3                        : 8;
	uint64_t size4                        : 8;
	uint64_t size5                        : 8;
	uint64_t size6                        : 8;
	uint64_t size7                        : 8;
#endif
	} s;
	struct cvmx_pko_reg_min_pkt_s         cn68xx;
	struct cvmx_pko_reg_min_pkt_s         cn68xxp1;
};

extern void *memset(void *, int, size_t);

void
bzero (void *to, size_t count)
{
  memset (to, 0, count);
}

#define	isalnum(ch)	(isalpha(ch) || isdigit(ch))
#define	isalpha(ch)	(isupper(ch) || islower(ch))
#define	isdigit(ch)	((ch) >= '0' && (ch) <= '9')
#define	islower(ch)	((ch) >= 'a' && (ch) <= 'z')
#define	isspace(ch)	(((ch) == ' ') || ((ch) == '\r') || ((ch) == '\n') || \
			((ch) == '\t') || ((ch) == '\f'))
#define	isupper(ch)	((ch) >= 'A' && (ch) <= 'Z')
#define	isxdigit(ch)	(isdigit(ch) || ((ch) >= 'a' && (ch) <= 'f') || \
			((ch) >= 'A' && (ch) <= 'F'))

#if defined(illumos) && (defined(_KERNEL) || defined(_BOOT))

#define	DIGIT(x)	\
	(isdigit(x) ? (x) - '0' : islower(x) ? (x) + 10 - 'a' : (x) + 10 - 'A')

inet_aton(cp, addr)
	register const char *cp;
	struct in_addr *addr;
{
}

#define PTR		void *
#define PTRCONST	void *const
#define LONG_DOUBLE	long double

typedef size_t rsize_t;


errno_t
memset_s(void *s, rsize_t smax, int c, rsize_t n)
{
	errno_t ret;
	rsize_t lim;
	unsigned char v;
	volatile unsigned char *dst;

	ret = EINVAL;
	lim = n < smax ? n : smax;
	v = (unsigned char)c;
	dst = (unsigned char *)s;
	if (s == NULL) {
		__throw_constraint_handler_s("memset_s : s is NULL", ret);
	} else if (smax > RSIZE_MAX) {
		__throw_constraint_handler_s("memset_s : smax > RSIZE_MAX",
		    ret);
	} else if (n > RSIZE_MAX) {
		__throw_constraint_handler_s("memset_s : n > RSIZE_MAX", ret);
	} else {
		while (lim > 0)
			dst[--lim] = v;
		if (n > smax) {
			__throw_constraint_handler_s("memset_s : n > smax",
			    ret);
		} else {
			ret = 0;
		}
	}
	return (ret);
}

#   define SIZE_MAX		(4294967295U)

#define	EINVAL		22		/* Invalid argument */
#define	ENFILE		23		/* Too many open files in system */
#define	EMFILE		24		/* Too many open files */
#define	ENOTTY		25		/* Inappropriate ioctl for device */

APU_DECLARE(apr_status_t) apr_crypto_memzero(void *buffer, apr_size_t size)
{
#if defined(WIN32)
    SecureZeroMemory(buffer, size);
#elif defined(HAVE_MEMSET_S)
    if (size) {
        return memset_s(buffer, (rsize_t)size, 0, (rsize_t)size);
    }
#elif defined(HAVE_EXPLICIT_BZERO)
    explicit_bzero(buffer, size);
#elif defined(HAVE_WEAK_SYMBOLS)
    apr__memzero_explicit(buffer, size);
#else
    apr_size_t i;
    volatile unsigned char *volatile ptr = buffer;
    for (i = 0; i < size; ++i) {
        ptr[i] = 0;
    }
#endif
    return APR_SUCCESS;
}


/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
size_t
strlcpy(char *dst, const char *src, size_t siz)
{
	char *d = dst;
	const char *s = src;
	size_t n = siz;

	/* Copy as many bytes as will fit */
	if (n != 0 && --n != 0) {
		do {
			if ((*d++ = *s++) == 0)
				break;
		} while (--n != 0);
	}

	/* Not enough room in dst, add NUL and traverse rest of src */
	if (n == 0) {
		if (siz != 0)
			*d = '\0';		/* NUL-terminate dst */
		while (*s++)
			;
	}

	return(s - src - 1);	/* count does not include NUL */
}

