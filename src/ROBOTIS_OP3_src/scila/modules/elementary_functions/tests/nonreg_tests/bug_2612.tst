// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) ????-2008 - INRIA
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- CLI SHELL MODE -->

ref = ["abc","bxe";"abc","aa"];
A=gsort(['abc','abd';'aa','bxe'],'c');
if ref <> A then pause,end
