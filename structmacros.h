#ifndef _STRUCTMACROS_INCLUDED
#define _STRUCTMACROS_INCLUDED

;-----------------------------------------------------------------------------
; Macros for defining structures in assembly
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


_strm_field_offset_ = -999999999

.macro .stroffsets
_strm_field_offset_ = 0
.endm

.macro .field name:req, size:req
\name = (_strm_field_offset_)
_strm_field_offset_ = (_strm_field_offset_) + (\size)
.endm

.macro .strend name:req
\name = (_strm_field_offset_)
_strm_field_offset_ = -999999999
.endm


;-----------------------------------------------------------------------------

#endif /* _STRUCTMACROS_INCLUDED */
