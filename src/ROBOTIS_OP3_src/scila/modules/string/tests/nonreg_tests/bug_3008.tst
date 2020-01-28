// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2008 - INRIA - Allan CORNET
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- CLI SHELL MODE -->

// <-- Non-regression test for bug 3008 -->
//
// <-- Bugzilla URL -->
// http://bugzilla.scilab.org/show_bug.cgi?id=3008
//

ref = ['|a|';'|b|'];
res = "|"+tokens("a b ")+"|";
if size(res,'*') <> 2 then pause,end;
if strcmp(ref,res) <> 0 then pause,end;


ref = ['|a|';'|b|'];
res = "|"+tokens("aibi",'i')+"|";
if size(res,'*') <> 2 then pause,end;
if strcmp(ref,res) <> 0 then pause,end;

