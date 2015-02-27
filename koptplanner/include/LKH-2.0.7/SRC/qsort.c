/*
   J. L. Bentley and M. D. McIlroy. Engineering a sort function.
   Software---Practice and Experience, 23(11):1249-1265.
*/

#define SWAPINIT(a, es) swaptype =                            \
    (a - (char*) 0) % sizeof(long) || es % sizeof(long) ? 2 : \
    es == sizeof(long) ? 0 : 1;
#define swapcode(TYPE, parmi, parmj, n) {  \
    long i = (n) / sizeof(TYPE);           \
    register TYPE *pi = (TYPE *) (parmi);  \
    register TYPE *pj = (TYPE *) (parmj);  \
    do {                                   \
        register TYPE t = *pi;             \
        *pi++ = *pj;                       \
        *pj++ = t;                         \
    } while (--i > 0);                     \
}
void swapfunc(char *a, char *b, int n, int swaptype)
{
    if (swaptype <= 1) {
        swapcode(long, a, b, n)
    } else
         swapcode(char, a, b, n)
}
#define swap(a, b)                         \
    if (swaptype == 0) {                   \
        long t = * (long *) (a);           \
        * (long *) (a) = * (long *) (b);   \
        * (long *) (b) = t;                \
    } else                                 \
        swapfunc(a, b, es, swaptype)
#define vecswap(a, b, n) if (n > 0) swapfunc(a, b, n, swaptype)
#define min(x, y) ((x)<=(y) ? (x) : (y))
char *med3(char *a, char *b, char *c,
           int (*cmp) (const void *, const void *))
{
    return cmp(a, b) < 0 ? (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a))
        : (cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c));
}

void qsort(void *base, unsigned n, int es,
           int (*cmp) (const void *, const void *))
{
    char *pa, *pb, *pc, *pd, *pl, *pm, *pn;
    int d, r, swaptype;
    char *a = (char *) base;

    SWAPINIT(a, es);
    if (n < 7) {                /* Insertion sort on small arrays */
        for (pm = a + es; pm < a + n * es; pm += es)
            for (pl = pm; pl > a && cmp(pl - es, pl) > 0; pl -= es)
                swap(pl, pl - es);
        return;
    }
    pm = a + (n / 2) * es;
    if (n > 7) {
        pl = a;
        pn = a + (n - 1) * es;
        if (n > 40) {           /* On big arrays, pseudomedian of 9 */
            d = (n / 8) * es;
            pl = med3(pl, pl + d, pl + 2 * d, cmp);
            pm = med3(pm - d, pm, pm + d, cmp);
            pn = med3(pn - 2 * d, pn - d, pn, cmp);
        }
        pm = med3(pl, pm, pn, cmp);     /* On mid arrays, med of 3 */
    }
    swap(a, pm);                /* On tiny arrays, partition around middle */
    pa = pb = a + es;
    pc = pd = a + (n - 1) * es;
    for (;;) {
        while (pb <= pc && (r = cmp(pb, a)) <= 0) {
            if (r == 0) {
                swap(pa, pb);
                pa += es;
            }
            pb += es;
        }
        while (pb <= pc && (r = cmp(pc, a)) >= 0) {
            if (r == 0) {
                swap(pc, pd);
                pd -= es;
            }
            pc -= es;
        }
        if (pb > pc)
            break;
        swap(pb, pc);
        pb += es;
        pc -= es;
    }
    pn = a + n * es;
    r = min(pa - a, pb - pa);
    vecswap(a, pb - r, r);
    r = min(pd - pc, pn - pd - es);
    vecswap(pb, pn - r, r);
    if ((r = pb - pa) > es)
        qsort(a, r / es, es, cmp);
    if ((r = pd - pc) > es)
        qsort(pn - r, r / es, es, cmp);
}
