// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2012 - DIGITEO - Alexandre HERISSE
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- XCOS TEST -->
//
// <-- Non-regression test for bug 10781 -->
//
// <-- Bugzilla URL -->
// http://bugzilla.scilab.org/show_bug.cgi?id=10781
//
// <-- Short Description -->
// DFlipflop should not report problem in port size or type

assert_checktrue(importXcosDiagram(SCI + "/modules/xcos/tests/nonreg_tests/bug_10781.zcos"));

// compile and simulate
xcos_simulate(scs_m, 4);

lastQ=double(Q.values($));
lastnonQ=double(nonQ.values($));

assert_checkequal(lastQ, 1);
assert_checkequal(lastnonQ, 0);
