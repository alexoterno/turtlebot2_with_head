// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2013 - Scilab Enterprises - Paul Bignier
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================
//
// <-- CLI SHELL MODE -->
//
// <-- Non-regression test for bug 11001 -->
//
// <-- Bugzilla URL -->
// http://bugzilla.scilab.org/show_bug.cgi?id=11001
//
// <-- Short Description -->
// exists and isdef did not recognize primitives.

assert_checkequal(exists("who"), 1);
assert_checkequal(exists("Who"), 0);
assert_checkequal(exists("exp"), 1);
assert_checkequal(exists("eXp"), 0);

assert_checkequal(isdef("who"), %t);
assert_checkequal(isdef("Who"), %f);
assert_checkequal(isdef("exp"), %t);
assert_checkequal(isdef("eXp"), %f);
