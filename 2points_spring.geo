lpart = 1.;
Point(13) = { 1., 0., 0., lpart};
Point(14) = { 1., 0., -1., lpart};
//+
Line(21) = {13, 14};
//+
Physical Point(1000) = {13};
//+
Physical Point(2000) = {14};
//+
Physical Line(300) = {21};
