#ifndef __BITS_H__
#define __BITS_H__

#define _ones(nbits)                    ((1UL<<nbits)-1)
#define _mask(nbits,lshift)             (_ones(nbits) << lshift)
#define _get(nbits,lshift,value)        (((value) >> lshift) & _ones(nbits))
#define _field(nbits,lshift,value)      ((((uint32_t)value) << lshift) & _mask(nbits,lshift))

#define ones(field)                     _ones(  field##_NBITS )
#define mask(field)                     _mask(  field##_NBITS, field##_LSHIFT )
#define get(field,value)                _get(   field##_NBITS, field##_LSHIFT, value)
#define field(field,value)              _field( field##_NBITS, field##_LSHIFT, value)

#endif
