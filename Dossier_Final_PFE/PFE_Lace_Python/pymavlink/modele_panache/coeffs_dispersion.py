def calc_sigmas_briggs(STABILITE, ZONE, x):
    sy1 = 0
    sy2 = 0
    sz1 = 0
    sz2 = 0
    sz3 = 0

    if ZONE == 'urbaine':
        if STABILITE == 'A':
            sy1 = 0.22
            sy2 = 0.0001
            sz1 = 0.24
            sz2 = 0.001
            sz3 = 0.5
        elif STABILITE == 'B':
            sy1 = 0.16
            sy2 = 0.0001
            sz1 = 0.24
            sz2 = 0.001
            sz3 = 0.5
        elif STABILITE == 'C':
            sy1 = 0.11
            sy2 = 0.0001
            sz1 = 0.2
            sz2 = 0
            sz3 = 0
        elif STABILITE == 'D':
            sy1 = 0.08
            sy2 = 0.0001
            sz1 = 0.14
            sz2 = 0.0003
            sz3 = -0.5
        elif STABILITE == 'E':
            sy1 = 0.06
            sy2 = 0.0001
            sz1 = 0.08
            sz2 = 0.0015
            sz3 = -0.5
        elif STABILITE == 'F':
            sy1 = 0.04
            sy2 = 0.0001
            sz1 = 0.08
            sz2 = 0.0015
            sz3 = -0.5
    elif ZONE == 'rurale':
        if STABILITE == 'A':
            sy1 = 0.22
            sy2 = 0.0001
            sz1 = 0.2
            sz2 = 0
            sz3 = 0
        elif STABILITE == 'B':
            sy1 = 0.16
            sy2 = 0.0001
            sz1 = 0.12
            sz2 = 0
            sz3 = 0
        elif STABILITE == 'C':
            sy1 = 0.11
            sy2 = 0.0001
            sz1 = 0.08
            sz2 = 0.0002
            sz3 = -0.5
        elif STABILITE == 'D':
            sy1 = 0.08
            sy2 = 0.0001
            sz1 = 0.06
            sz2 = 0.0015
            sz3 = -0.5
        elif STABILITE == 'E':
            sy1 = 0.06
            sy2 = 0.0001
            sz1 = 0.03
            sz2 = 0.0003
            sz3 = -1
        elif STABILITE == 'F':
            sy1 = 0.04
            sy2 = 0.0001
            sz1 = 0.016
            sz2 = 0.0003
            sz3 = -1

    sig_y = (sy1 * x) / (1 + sy2 * x)**0.5  # sqrt(1+sy2*x) équivaut à (1+sy2*x)^0.5
    sig_z = (sz1 * x) * (1 + sz2 * x)**sz3

    return sig_y, sig_z
