
/*
 * knurledFinishLib.scad
 * 
 * Written by aubenc @ Thingiverse
 *
 * This script is licensed under the Public Domain license.
 *
 * http://www.thingiverse.com/thing:9095
 *
 * Usage:
 *
 *    knurled_cyl( Knurled cylinder height,
 *                 Knurled cylinder outer diameter,
 *                 Knurl polyhedron width,
 *                 Knurl polyhedron height,
 *                 Knurl polyhedron depth,
 *                 Cylinder ends smoothed height,
 *                 Knurled surface smoothing amount );
 */
module knurled_cyl(chg, cod, cwd, csh, cdp, fsh, smt)
{
    cord=(cod+cdp+cdp*smt/100)/2;
    cird=cord-cdp;
    cfn=round(2*cird*PI/cwd);
    clf=360/cfn;
    crn=ceil(chg/csh);

    intersection()
    {
        shape(fsh, cird, cord-cdp*smt/100, cfn*4, chg);

        translate([0,0,-(crn*csh-chg)/2])
          knurled_finish(cord, cird, clf, csh, cfn, crn);
    }
}

module shape(hsh, ird, ord, fn4, hg)
{
        union()
        {
            cylinder(h=hsh, r1=ird, r2=ord, $fn=fn4, center=false);

            translate([0,0,hsh-0.002])
              cylinder(h=hg-2*hsh+0.004, r=ord, $fn=fn4, center=false);

            translate([0,0,hg-hsh])
              cylinder(h=hsh, r1=ord, r2=ird, $fn=fn4, center=false);
        }

}

module knurled_finish(ord, ird, lf, sh, fn, rn)
{
    for(j=[0:rn-1])
    
    {
      h0=sh*j;
      h1=sh*(j+1/2);
      h2=sh*(j+1);
        for(i=[0:fn-1])
        
        {
          lf0=lf*i;
        lf1=lf*(i+1/2);
        lf2=lf*(i+1);
            polyhedron(
                points=[
                     [ 0,0,h0],
                     [ ord*cos(lf0), ord*sin(lf0), h0],
                     [ ird*cos(lf1), ird*sin(lf1), h0],
                     [ ord*cos(lf2), ord*sin(lf2), h0],

                     [ ird*cos(lf0), ird*sin(lf0), h1],
                     [ ord*cos(lf1), ord*sin(lf1), h1],
                     [ ird*cos(lf2), ird*sin(lf2), h1],

                     [ 0,0,h2],
                     [ ord*cos(lf0), ord*sin(lf0), h2],
                     [ ird*cos(lf1), ird*sin(lf1), h2],
                     [ ord*cos(lf2), ord*sin(lf2), h2]
                    ],
                faces=[
                     [0,1,2],[2,3,0],
                     [1,0,4],[4,0,7],[7,8,4],
                     [8,7,9],[10,9,7],
                     [10,7,6],[6,7,0],[3,6,0],
                     [2,1,4],[3,2,6],[10,6,9],[8,9,4],
                     [4,5,2],[2,5,6],[6,5,9],[9,5,4]
                    ],
                convexity=5);
         }
    }
}
$fn = 64;
union()
{
  difference()
  {
    union()
    {
      rotate(a = [5, 0, 0], v = undef)
      {
        union()
        {
          translate(v = [0, -22, -7])
          {
            union()
            {
              translate(v = [-85.72500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-66.67500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-47.62500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-28.575000000000003, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-9.525000000000006, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [9.524999999999991, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [28.575000000000003, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [47.624999999999986, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
          translate(v = [0, 12, -7])
          {
            union()
            {
              translate(v = [-85.72500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-66.67500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-47.62500000000001, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-28.575000000000003, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [-9.525000000000006, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [9.524999999999991, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [28.575000000000003, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
              translate(v = [47.624999999999986, 0, 16])
              {
                union()
                {
                  union()
                  {
                    translate(v = [0, 7.95, 2])
                    {
                      cube(size = [17.4, 1.5, 4], center = true);
                    }
                    translate(v = [7.95, 0, 2])
                    {
                      cube(size = [1.5, 17.4, 4], center = true);
                    }
                    hull()
                    {
                      translate(v = [7.2, 0, 1])
                      {
                        rotate(a = [90, 0, 0], v = undef)
                        {
                          cylinder(h = 2.75, r = 1, center = true);
                        }
                      }
                      translate(v = [7.95, 0, 2])
                      {
                        cube(size = [1.5, 2.75, 4], center = true);
                      }
                    }
                  }
                  mirror(v = [0, 1, 0])
                  {
                    mirror(v = [1, 0, 0])
                    {
                      union()
                      {
                        translate(v = [0, 7.95, 2])
                        {
                          cube(size = [17.4, 1.5, 4], center = true);
                        }
                        translate(v = [7.95, 0, 2])
                        {
                          cube(size = [1.5, 17.4, 4], center = true);
                        }
                        hull()
                        {
                          translate(v = [7.2, 0, 1])
                          {
                            rotate(a = [90, 0, 0], v = undef)
                            {
                              cylinder(h = 2.75, r = 1, center = true);
                            }
                          }
                          translate(v = [7.95, 0, 2])
                          {
                            cube(size = [1.5, 2.75, 4], center = true);
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
          difference()
          {
            translate(v = [-19.05, -22, 14])
            {
              cube(size = [156.4, 24.05, 12], center = true);
            }
            translate(v = [-19.05, -22, 20])
            {
              cube(size = [154.4, 21.05, 14], center = true);
            }
            translate(v = [0, -22, -7])
            {
              union()
              {
                translate(v = [-85.72500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-66.67500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-47.62500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-28.575000000000003, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-9.525000000000006, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [9.524999999999991, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [28.575000000000003, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [47.624999999999986, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
              }
            }
          }
          difference()
          {
            translate(v = [-19.05, 12, 14])
            {
              cube(size = [156.4, 24.05, 12], center = true);
            }
            translate(v = [-19.05, 12, 20])
            {
              cube(size = [154.4, 21.05, 14], center = true);
            }
            translate(v = [0, 12, -7])
            {
              union()
              {
                translate(v = [-85.72500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-66.67500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-47.62500000000001, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-28.575000000000003, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [-9.525000000000006, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [9.524999999999991, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [28.575000000000003, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
                translate(v = [47.624999999999986, 0, 16])
                {
                  cube(size = [14.4, 14.4, 16], center = true);
                }
              }
            }
            translate(v = [77.5, 25, 19])
            {
              union()
              {
                cube(size = [30, 12, 3], center = true);
                translate(v = [2, 0.25, -4])
                {
                  cube(size = [40, 12.5, 8], center = true);
                }
              }
            }
          }
        }
      }
      translate(v = [97.75, 32.5, 0])
      {
        union()
        {
          cylinder(h = 10, r = 5, center = true);
          translate(v = [0, 0, 5])
          {
            sphere(r = 5);
          }
        }
      }
      translate(v = [-97.75, -36.5, 0])
      {
        union()
        {
          cylinder(h = 10, r = 5, center = true);
          translate(v = [0, 0, 5])
          {
            sphere(r = 5);
          }
        }
      }
      translate(v = [-97.75, 32.5, 0])
      {
        union()
        {
          cylinder(h = 10, r = 5, center = true);
          translate(v = [0, 0, 5])
          {
            sphere(r = 5);
          }
        }
      }
      translate(v = [97.75, -36.5, 0])
      {
        union()
        {
          cylinder(h = 10, r = 5, center = true);
          translate(v = [0, 0, 5])
          {
            sphere(r = 5);
          }
        }
      }
      difference()
      {
        union()
        {
          translate(v = [0, 0, -5])
          {
            rotate(a = [5, 0, 0], v = undef)
            {
              multmatrix(m = [[1, 0, 0, 0], [0, 1, 0.08715574274765817, -2], [0, 0, 1, 0]])
              {
                difference()
                {
                  minkowski()
                  {
                    cube(size = [195.5, 70, 40], center = true);
                    sphere(r = 5);
                  }
                  minkowski()
                  {
                    cube(size = [195.5, 70, 40], center = true);
                    sphere(r = 1);
                  }
                }
              }
            }
          }
          rotate(a = [5, 0, 0], v = undef)
          {
            union()
            {
              translate(v = [-19.05, -5, 14.5])
              {
                cube(size = [156.4, 12, 3], center = true);
              }
              translate(v = [-19.05, 29, 14.5])
              {
                cube(size = [156.4, 12, 3], center = true);
              }
            }
          }
        }
        rotate(a = [5, 0, 0], v = undef)
        {
          union()
          {
            translate(v = [-19.05, -22, 15])
            {
              cube(size = [154.4, 21.05, 14], center = true);
            }
            translate(v = [-19.05, 12, 15])
            {
              cube(size = [154.4, 21.05, 14], center = true);
            }
            translate(v = [0, -7, 0])
            {
              union()
              {
                translate(v = [-85.72500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-66.67500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-47.62500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-28.575000000000003, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-9.525000000000006, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [9.524999999999991, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [28.575000000000003, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [47.624999999999986, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
              }
            }
            translate(v = [0, 28, 0])
            {
              union()
              {
                translate(v = [-85.72500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-66.67500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-47.62500000000001, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-28.575000000000003, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [-9.525000000000006, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [9.524999999999991, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [28.575000000000003, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
                translate(v = [47.624999999999986, 0, 16])
                {
                  cylinder(h = 30, r = 2.55, center = true);
                }
              }
            }
            translate(v = [79, -15, 20])
            {
              cylinder(h = 50, r = 3.6, center = true);
            }
            translate(v = [79, 25, 2])
            {
              cube(size = [10, 40, 5], center = true);
            }
            translate(v = [77.5, 25, 19])
            {
              union()
              {
                cube(size = [30, 12, 3], center = true);
                translate(v = [2, 0.25, -4])
                {
                  cube(size = [40, 12.5, 8], center = true);
                }
              }
            }
            translate(v = [79, -15, 8])
            {
              cube(size = [13.5, 15.5, 20], center = true);
            }
          }
        }
      }
    }
    rotate(a = [5, 0, 0], v = undef)
    {
      translate(v = [79.5, 6, 19])
      {
        linear_extrude(height = 5, center = false, convexity = undef, twist = undef, slices = undef, scale = 1, $fn = 20)
        {
          text(text = "SEEQ", font = "Anurati", size = 9, halign = "center");
        }
      }
    }
    translate(v = [-17, 36.5, 14])
    {
      rotate(a = [90, 0, 180], v = undef)
      {
        linear_extrude(height = 5, center = false, convexity = undef, twist = undef, slices = undef, scale = 1, $fn = 20)
        {
          text(text = "MIDI IN", font = "Iosevka:style=Heavy Extended", size = 4, halign = "center");
        }
      }
    }
    translate(v = [-55, 36.5, 14])
    {
      rotate(a = [90, 0, 180], v = undef)
      {
        linear_extrude(height = 5, center = false, convexity = undef, twist = undef, slices = undef, scale = 1, $fn = 20)
        {
          text(text = "MIDI OUT", font = "Iosevka:style=Heavy Extended", size = 4, halign = "center");
        }
      }
    }
    translate(v = [40, 36.5, 14])
    {
      rotate(a = [90, 0, 180], v = undef)
      {
        linear_extrude(height = 5, center = false, convexity = undef, twist = undef, slices = undef, scale = 1, $fn = 20)
        {
          text(text = "CV", font = "Iosevka:style=Heavy Extended", size = 4, halign = "center");
        }
      }
    }
    translate(v = [20, 36.5, 14])
    {
      rotate(a = [90, 0, 180], v = undef)
      {
        linear_extrude(height = 5, center = false, convexity = undef, twist = undef, slices = undef, scale = 1, $fn = 20)
        {
          text(text = "GATE", font = "Iosevka:style=Heavy Extended", size = 4, halign = "center");
        }
      }
    }
    translate(v = [79.5, 29, 3])
    {
      cube(size = [25, 16, 12], center = true);
    }
    translate(v = [0, 0, -30])
    {
      cube(size = [381, 100, 60], center = true);
    }
    translate(v = [40, 32, 5])
    {
      rotate(a = [0, 0, 90], v = undef)
      {
        union()
        {
          rotate(a = [0, 90, 0], v = undef)
          {
            translate(v = [0, 0, 6])
            {
              cylinder(h = 12, r = 3, center = true);
            }
          }
          cube(size = [8, 12, 10], center = true);
        }
      }
    }
    translate(v = [20, 32, 5])
    {
      rotate(a = [0, 0, 90], v = undef)
      {
        union()
        {
          rotate(a = [0, 90, 0], v = undef)
          {
            translate(v = [0, 0, 6])
            {
              cylinder(h = 12, r = 3, center = true);
            }
          }
          cube(size = [8, 12, 10], center = true);
        }
      }
    }
    translate(v = [-17, 35, 5])
    {
      rotate(a = [90, 0, 0], v = undef)
      {
        cylinder(h = 20, r = 8, center = true);
      }
    }
    translate(v = [-55, 35, 5])
    {
      rotate(a = [90, 0, 0], v = undef)
      {
        cylinder(h = 20, r = 8, center = true);
      }
    }
    translate(v = [97.75, 33, 0])
    {
      translate(v = [0, 0, 3])
      {
        cylinder(h = 6.1, r = 2.3, center = true);
      }
    }
    translate(v = [97.75, -37, 0])
    {
      translate(v = [0, 0, 3])
      {
        cylinder(h = 6.1, r = 2.3, center = true);
      }
    }
    translate(v = [-97.75, 33, 0])
    {
      translate(v = [0, 0, 3])
      {
        cylinder(h = 6.1, r = 2.3, center = true);
      }
    }
    translate(v = [-97.75, -37, 0])
    {
      translate(v = [0, 0, 3])
      {
        cylinder(h = 6.1, r = 2.3, center = true);
      }
    }
    translate(v = [-97.75, 0, 15])
    {
      rotate(a = [95, 0, 0], v = undef)
      {
        difference()
        {
          cylinder(h = 100, r = 50, center = true);
          cylinder(h = 101, r = 5, center = true);
          translate(v = [60, 0, 0])
          {
            cube(size = [120, 120, 120], center = true);
          }
          translate(v = [0, -60, 0])
          {
            cube(size = [120, 120, 120], center = true);
          }
        }
      }
    }
  }
}
