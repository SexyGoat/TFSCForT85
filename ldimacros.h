#ifndef _LDIMACROS_INCLUDED
#define _LDIMACROS_INCLUDED

;-----------------------------------------------------------------------------
; 16-bit load pseudo-instruction macros for 8-bit AVR MCUs
;-----------------------------------------------------------------------------
;
; (c) Copyright 2022, Daniel Neville (creamygoat@gmail.com)
;
; This file is part of TFSCForT85.
;
; TFSCForT85 is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; TFSCForT85 is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with TFSCForT85. If not, see <https://www.gnu.org/licenses/>.
;
;-----------------------------------------------------------------------------


; Load high & low 8-bit registers with one 16-bit word

.macro  ldhl      a:req, b:req, c:req
        ldi       \a, hi8(\c)
        ldi       \b, lo8(\c)
.endm


;-----------------------------------------------------------------------------


; Load immediate word

.macro  ldiw    a:req, b:req
  .ifc \a, r17:r16
        ldhl    r17, r16, \b
  .else
  .ifc \a, r19:r18
        ldhl    r19, r18, \b
  .else
  .ifc \a, r21:r20
        ldhl    r21, r20, \b
  .else
  .ifc \a, r23:r22
        ldhl    r23, r22, \b
  .else
  .ifc \a, r25:r24
        ldhl    r25, r24, \b
  .else
  .ifc \a, r27:r26
        ldhl    XH, XL, \b
  .else
  .ifc \a, r29:r28
        ldhl    YH, YL, \b
  .else
  .ifc \a, r31:r30
        ldhl    ZH, ZL, \b
  .else
  .ifc \a, X
        ldhl    XH, XL, \b
  .else
  .ifc \a, Y
        ldhl    YH, YL, \b
  .else
  .ifc \a, Z
        ldhl    ZH, ZL, \b
  .else
    .error "Expected LDIW R, x where R is X, Y, Z or a register pair."
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
.endm


;-----------------------------------------------------------------------------


; Load high & low 8-bit registers with one 16-bit program memory word address.

.macro  ldpmhl    a:req, b:req, c:req
        ldi       \a, pm_hi8(\c)
        ldi       \b, pm_lo8(\c)
.endm


;-----------------------------------------------------------------------------


; Load immediate word address in program memory (for ijmp, icall etc.)

.macro  ldip    a:req, b:req
  .ifc \a, r17:r16
        ldpmhl  r17, r16, \b
  .else
  .ifc \a, r19:r18
        ldpmhl  r19, r18, \b
  .else
  .ifc \a, r21:r20
        ldpmhl  r21, r20, \b
  .else
  .ifc \a, r23:r22
        ldpmhl  r23, r22, \b
  .else
  .ifc \a, r25:r24
        ldpmhl  r25, r24, \b
  .else
  .ifc \a, r27:r26
        ldpmhl  XH, XL, \b
  .else
  .ifc \a, r29:r28
        ldpmhl  YH, YL, \b
  .else
  .ifc \a, r31:r30
        ldpmhl  ZH, ZL, \b
  .else
  .ifc \a, X
        ldpmhl  XH, XL, \b
  .else
  .ifc \a, Y
        ldpmhl  YH, YL, \b
  .else
  .ifc \a, Z
        ldpmhl  ZH, ZL, \b
  .else
    .error "Expected LDIP R, x where R is X, Y, Z or a register pair."
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
  .endif
.endm


;-----------------------------------------------------------------------------

#endif /* _LDIMACROS_INCLUDED */
