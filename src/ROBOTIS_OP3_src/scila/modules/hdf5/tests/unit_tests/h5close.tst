// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2012 - SCILAB ENTERPRISES - Simon GARESTE
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================
//
// <-- CLI SHELL MODE -->

msgerr = msprintf(gettext("%s: Wrong type for input argument #%d: A H5Object expected.\n"), "h5close", 1);
assert_checkerror("h5close(42)",msgerr,999);
assert_checkerror("h5close(""42"")",msgerr,999);

w = [1 2;3 4];

save(TMPDIR + "/w.sod", "w");

a = h5open(TMPDIR + "/w.sod", "r");
h5close(a);
msgerr = msprintf(gettext("%s: Can not print H5Object: invalid object.\n"), "%H5Object_p");
assert_checkerror("disp(a)",msgerr,999);

a = h5open(TMPDIR + "/w.sod", "r");
dataset = a.root.w;
attr = dataset.SCILAB_Class;
h5close(dataset);
msgerr = msprintf(gettext("%s: Can not print H5Object: invalid object.\n"), "%H5Object_p");
assert_checkerror("disp(attr)",msgerr,999);

//a still open
dataset = a.root.w;
attr = dataset.SCILAB_Class;
//dataset is a descendant of a, so closing a will close dataset
h5close(a);
assert_checkerror("disp(a)",msgerr,999);
assert_checkerror("disp(dataset)",msgerr,999);
assert_checkerror("disp(attr)",msgerr,999);


assert_checkequal(deletefile(TMPDIR+"/w.sod"),%T);
