#ifndef __COMMON_H__
#define __COMMON_H__

const double INVALID_POINT = 0.0;

#define POW2(x) ((x)*(x))
#define DIS2(p,q) (POW2((p).x-(q).x)+POW2((p).y-(q).y)+POW2((p).z-(q).z))
#define IS_VALID_POINT(p)  ((p).z!=INVALID_POINT)
#define SET_INVALID_POINT(p)  ((p).z = INVALID_POINT)

#endif
