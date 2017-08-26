/* -----------------------
   File 2points_spring.pro
----------------------- */

/*======================
  Geometrical Entities
======================*/

Group {
   // Boundary
   Pt0 = Region[ 1000 ];
   Pt1 = Region[ 2000 ];

   // The Domain (the line between Pt1 and Pt2)
   Line = Region[ 300 ];
   
}



/*=====================
   Some Functions
=====================*/

Function {
   // Initial State
   InitialState[Line] = Z[]*4. ;
   rht[Line] = Cos[Z[]*4.] ;
   nul[Line] = 0.;
   Fx = 1000 ;
   Fy= 10000 ;
   Fz = 100 ;
}



/*===============================
  Constraints and Initial State
===============================*/

Constraint {
   // Initial State
   { Name InitialData; Type Init;
      Case {
         { Region Line; Value 0.; }
      }
   }
   // Boundary control
   { Name P0; Type Assign;
      Case {
         { Region Pt0; Value 0.; }
      }
   }
}



/*====================
  Functional Spaces
====================*/

FunctionSpace {
   { Name depz; Type Form0;
      BasisFunction{
      { Name wn; NameOfCoef vn; Function BF_Node;
         Support Line; Entity NodesOf[All];}
      }
      Constraint {
      { NameOfCoef vn; EntityType NodesOf;
         NameOfConstraint P0; }
      { NameOfCoef vn; EntityType NodesOf;
         NameOfConstraint InitialData; }
      }
   }
}

/*====================
    Jacobian
====================*/

Jacobian {
  { Name JVol ;
    Case {
      { Region All ; Jacobian Vol ; }
    }
  }
  { Name JSur ;
    Case {
      { Region All ; Jacobian Sur ; }
    }
  }
  { Name JLin ;
    Case {
      { Region All ; Jacobian Lin ; }
    }
  }
}

/*======================
   Integral Parameters
======================*/

Integration {
  { Name I1 ;
    Case {
      { Type Gauss ;
        Case {
          { GeoElement Point ; NumberOfPoints  1 ; }
          { GeoElement Line ; NumberOfPoints  4 ; }
          { GeoElement Triangle ; NumberOfPoints  6 ; }
          { GeoElement Quadrangle ; NumberOfPoints 7 ; }
          { GeoElement Tetrahedron ; NumberOfPoints 15 ; }
          { GeoElement Hexahedron ; NumberOfPoints 34 ; }
        }
      }
    }
  }
}


/*======================
  Weak formulations
======================*/

Formulation {
   { Name ressort; Type FemEquation;
      Quantity{
         { Name u; Type Local; NameOfSpace depz;}
         { Name v; Type Local; NameOfSpace depz;}
         { Name w; Type Local; NameOfSpace depz;}
      }

   Equation{
      Galerkin{  [10000. * Dof{Grad u} , {Grad u} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [10000. * Dof{u} , {u} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [Fx  , {u} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [10000. * Dof{Grad v} , {Grad v} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [10000. * Dof{v} , {v} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [Fy  , {v} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [10000. * Dof{Grad w} , {Grad w} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [10000. * Dof{w} , {w} ];
         In Line; Jacobian JLin; Integration I1;}
      Galerkin{  [Fz  , {w} ];
         In Line; Jacobian JLin; Integration I1;}
    }
  }
}


/*======================
  Resolution
======================*/

Resolution {
   { Name SpringSolver;
      System{
         { Name SysSpring; NameOfFormulation ressort;}
      }
      Operation{
            // Initialisation
            InitSolution[SysSpring];
            Generate[SysSpring]; Solve[SysSpring];
      }
   }
}

/*==============
Post Processing
==============*/

PostProcessing {
   { Name SpringSolver; NameOfFormulation ressort; NameOfSystem SysSpring;
      Quantity{
         { Name z0; Value {Local{[{w}]; In Line; Jacobian JVol;}}}
	 { Name depl ; Value { Term { [ Vector[ {u}, {v}, {w} ]];
	                   In Line ; Jacobian JVol ; } } }
      }
   }
}

/*=============
Post Operation
=============*/
PostOperation {
   {Name Map_Omega; NameOfPostProcessing SpringSolver;
      Operation{
         Print[z0, OnElementsOf Line, File "z.pos"];
         Print[depl, OnElementsOf Line, File "deplacement.pos"];
      }
   }
}
