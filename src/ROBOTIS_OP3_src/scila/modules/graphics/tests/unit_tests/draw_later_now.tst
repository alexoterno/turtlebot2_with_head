// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2011 - DIGITEO - Bruno JOFRET
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- TEST WITH GRAPHIC -->

// test drawlater/drawnow behaviour

f = gcf();

drawlater();
assert_checkequal(f.immediate_drawing, "off");

drawnow();
assert_checkequal(f.immediate_drawing, "on");
