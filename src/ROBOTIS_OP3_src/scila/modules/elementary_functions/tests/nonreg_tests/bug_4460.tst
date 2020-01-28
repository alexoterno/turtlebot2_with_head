// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2009 - DIGITEO - Vincent COUVERT <vincent.couvert@scilab.org>
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- CLI SHELL MODE -->

// <-- Non-regression test for bug 4460 -->
//
// <-- Bugzilla URL -->
// http://bugzilla.scilab.org/show_bug.cgi?id=4460
//
// <-- Short Description -->
//    Cat does not work with character strings

A = "str1";
B = "str2";

if ~and(cat(1,A,B)==[A;B]) then pause; end

// Others tests added
A = %T;
B = %F;

if ~and(cat(1,A,B)==[A;B]) then pause; end
