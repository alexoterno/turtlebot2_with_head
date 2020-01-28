// =============================================================================
// Scilab ( http://www.scilab.org/ ) - This file is part of Scilab
// Copyright (C) 2013 - Scilab Enterprises - Bruno JOFRET
//
//  This file is distributed under the same license as the Scilab package.
// =============================================================================

// <-- XCOS TEST -->

// pre_xcos_simulate test
loadXcosLibs();

assert_checktrue(importXcosDiagram(SCI + "/modules/xcos/tests/unit_tests/cumsum.zcos"));

function continueSimulation = fail_pre_simulate(scs_m, needcompile)
    disp("Calling fail_pre_simulate");
    continueSimulation = %f;
endfunction

// Register function
pre_xcos_simulate = list("fail_pre_simulate");
xcos_simulate(scs_m, 4);
assert_checkfalse(isdef('cumsum_r'));

// Register function
clear pre_xcos_simulate
pre_xcos_simulate = fail_pre_simulate;
xcos_simulate(scs_m, 4);
assert_checkfalse(isdef('cumsum_r'));

function continueSimulation = analyze_pre_simulate(scs_m, needcompile)
// Retrieve all objects
    objs = scs_m.objs;

    links = 0;
    blocks = 0;
    other = 0;
// Count Links and Blocks
    for i = 1:size(objs)
        currentType = typeof(objs(i));
        select (currentType)
         case "Link"
          links = links + 1;
         case "Block"
          blocks = blocks + 1;
        else
            other = other + 1;
        end
    end

// Diplay Diagram analisys.
    disp("Diagram Analysis:")
    disp("Found "+string(blocks)+" Blocks.")
    disp("Found "+string(links)+" Links.")
    disp("Found "+string(other)+" Other component.")

// Continue Simulation
    continueSimulation = %T;
endfunction

// Register function
clear pre_xcos_simulate
pre_xcos_simulate = list("analyze_pre_simulate");
xcos_simulate(scs_m, 4);
assert_checktrue(isdef('cumsum_r'));

clear('cumsum_r');

// Register function
clear pre_xcos_simulate
pre_xcos_simulate = analyze_pre_simulate;
xcos_simulate(scs_m, 4);
assert_checktrue(isdef('cumsum_r'));
