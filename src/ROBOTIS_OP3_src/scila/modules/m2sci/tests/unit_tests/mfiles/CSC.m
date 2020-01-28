% Test file for function csc()
% Matlab version: 7.9.0.529 (R2009b)

% TEST 1
res1 = csc([]);
% TEST 2
res2 = csc(m2sciUnknownType([]));
% TEST 3
res3 = csc(m2sciUnknownDims([]));
% TEST 4
res4 = csc([1]);
% TEST 5
res5 = csc([1,2,3]);
% TEST 6
res6 = csc([1;2;3]);
% TEST 7
res7 = csc([1,2,3;4,5,6]);
% TEST 8
res8 = csc(m2sciUnknownType([1]));
% TEST 9
res9 = csc(m2sciUnknownType([1,2,3]));
% TEST 10
res10 = csc(m2sciUnknownType([1;2;3]));
% TEST 11
res11 = csc(m2sciUnknownType([1,2,3;4,5,6]));
% TEST 12
res12 = csc(m2sciUnknownDims([1]));
% TEST 13
res13 = csc(m2sciUnknownDims([1,2,3]));
% TEST 14
res14 = csc(m2sciUnknownDims([1;2;3]));
% TEST 15
res15 = csc(m2sciUnknownDims([1,2,3;4,5,6]));
% TEST 16
res16 = csc([i]);
% TEST 17
res17 = csc([i,2i,3i]);
% TEST 18
res18 = csc([i;2i;3i]);
% TEST 19
res19 = csc([i,2i,3i;4i,5i,6i]);
% TEST 20
res20 = csc(m2sciUnknownType([i]));
% TEST 21
res21 = csc(m2sciUnknownType([i,2i,3i]));
% TEST 22
res22 = csc(m2sciUnknownType([i;2i;3i]));
% TEST 23
res23 = csc(m2sciUnknownType([i,2i,3i;4i,5i,6i]));
% TEST 24
res24 = csc(m2sciUnknownDims([i]));
% TEST 25
res25 = csc(m2sciUnknownDims([i,2i,3i]));
% TEST 26
res26 = csc(m2sciUnknownDims([i;2i;3i]));
% TEST 27
res27 = csc(m2sciUnknownDims([i,2i,3i;4i,5i,6i]));
