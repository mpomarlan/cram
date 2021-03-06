;; Regression test GEOMETRIC for GSLL, automatically generated
;;
;; Copyright 2009 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :gsl)

(LISP-UNIT:DEFINE-TEST GEOMETRIC
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST (LIST 1 4 3 1 3 2 1 1 2 1 1))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :geometric :probability 0.4d0)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.24d0)
   (MULTIPLE-VALUE-LIST (GEOMETRIC-PDF 2 0.4d0)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.64d0)
   (MULTIPLE-VALUE-LIST (GEOMETRIC-P 2 0.4d0)))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.36d0)
   (MULTIPLE-VALUE-LIST (GEOMETRIC-Q 2 0.4d0))))
