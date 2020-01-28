// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2011 - DIGITEO - Allan CORNET
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================
//
// <-- CLI SHELL MODE -->
//
// <-- Non-regression test for bug 8836 -->
//
// <-- Bugzilla URL -->
// http://bugzilla.scilab.org/show_bug.cgi?id=8836
//
// <-- Short Description -->
// fileparts crashed scilab when matrix of strings was passed as input argument.
// Following the commit: https://codereview.scilab.org/#/c/11620/, fileparts 
// manages the matrix of strings.

assert_checktrue(execstr("fileparts(ls(""SCI/modules/fileio/macros/*.sci""))", "errcatch") == 0);
