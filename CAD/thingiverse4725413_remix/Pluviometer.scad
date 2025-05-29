/* This thing is modified from https://www.thingiverse.com/thing:4725413
** (c) 2025 by Wayne E Wright, https://www.thingiverse.com/wwrr55 
**
** The overall design, including the sources to the 3D models, the Arduino sketch, 
** the printed circuit board, etc. are published on github (https://github.com/w5xd/PacketRainGauge)
** The 3D printable STL files are published on thingiverse (and not on github).
**
** While the overall structure of the 4725413 scad file, Pluviometer.scad, is retained, none of the parts
** below is compatible with printed versions of the original. Each has been modified in
** some way from the original.
**
** The bucket is modified to accept a different magnet, 1/4" x 1/8" x 1/8"
** The threaded base/funnel joint is now implemented with the standard threads library: https://github.com/rcolyer/threads-scad
** The sensor and sensor_back are unused in my implementation
** The base is modified to have a built-in dual AA cell battery bracket
** AA cell holder to match the bracket is added.
** A flange to mate a certain T-post steel fence post is added.
**
** A companion FreeCAD file contains a design to place a hall effect sensor PCB
** at the base's sensor slot.
*/
use <threads.scad>

cable_holder = false;

view_as_assembled = 1; // [1:as assembled, 0:expoloded, 2:XZ cross seciton, 3:YZ cross section, 4:3D print, 5:intersect base&AA]
show_base = true;
show_base_supports = false;
show_flange = false;
show_cellAACover = true;
show_bucket = true;
show_funnel = true;
show_sensor = false;
show_sensor_back = false;

funnel_height_mm = 120;
funnel_inner_height_mm = 70;
base_diameter_mm = 120;

number8MachineScrewBoltHoleDiameter = 25.4 * .18;
number8MachineScrewSocketHeadDiameter = 25.4 * .285;
number8MachineScrewSocketHeadHeight = 25.4 * .17;
number8MachineNutHeight = 25.4 * .135;
number8MachineNutAcrossFlats = 25.4 * 11 / 32;

base_funnel_thread_diameter_mm = 105;
base_funnel_thread_height_mm = 14;
base_funnel_thread_pitch_height_mm = 2.5;
base_mount_hole_diam_mm = number8MachineScrewBoltHoleDiameter;
base_disk_thickness = 5;

fnval = $preview ? 12 : 120;
fn30val = $preview ? 10 : 30;
$fn = $preview ? 12 : 120;

module __x__ () {}

bucket_slot_width_mm = 24;

flange_start_z_mm = -23;
flange_fastener_diameter_mm = 18;
flange_fastener_height_mm = 3;
flange_transitionmm = 25.4 * 1.5;
flange_post_overlap = 25.4 * 1.5;
flange_post_cut_depth = 25.4 * 2;

tan30squared = tan(30) * tan(30); // for converting nut "across flats
acrossFlatsConvert = sqrt(1 + tan30squared);

module triangleR(a, b, h, c){
  linear_extrude(height = h, center = c, convexity = 10)
  resize([a, b])
  polygon([[0,0], [1,0], [0,1]], paths=[[0,1,2]]);
}

module triangleRa(a, angle, h, c){
  rotate([90, 0, 0])
  linear_extrude(height = h, center = c, convexity = 10)
  resize([a, a*tan(angle)])
  polygon([[0,0], [1,0], [0,1]], paths=[[0,1,2]]);
}

module triangleI(x, y, z, c){
  rotate([90, 0, 0])
  linear_extrude(height = y, center = c, convexity = 10)
  resize([x, z])
  polygon([[-1,0], [1,0], [0,1]], paths=[[0,1,2]]);
}

module bucket(){
  one4 = 25.4 * .25;
  one8 = one4 * .5;
  magClearance = .008 * 25.4;
  magY = 9;
  magZ = 18;
  color("cyan")
  difference(){
    union(){
      difference(){
        triangleI(70, 20, 25, true);
        
        translate([-70/4-1/2, 0, (25+1)/2]) cube([70/2, 20-2, 25-1], center = true);
        translate([70/4+1/2, 0, (25+1)/2]) cube([70/2, 20-2, 25-1], center = true);
      }
      // centering the bucket
      linear_extrude(height=1)
        square([5, bucket_slot_width_mm-.9], center=true);

      // magnet outer
      magOuterDx = one8 + one8;
      magOuterDy = one8 + one8/2;
      magOuterDz = one4+one8;

      magOuterYZProfile=[[magOuterDy/2, -magOuterDz/2], 
                [magOuterDy/2, magOuterDz/2],
                [-magOuterDy/2, magOuterDz/2],
                [-magOuterDy/2, -(magOuterDz + one4 + one8)/2]];
      translate([0, magY + one8/4, magZ])
      rotate([90,0,90])
      linear_extrude(height=magOuterDx, center=true)
          polygon(magOuterYZProfile);
    
      triangleI(25, 20, 15, true);
    }
    
    // pivot shaft hole
    translate([0, 0, 3]) rotate([90, 0, 0]) cylinder(d=3.25, h=20, center=true, $fn=fn30val);

    // magnet inner
    translate([0, magY, magZ])
    translate([0,one8/2])
      cube([one8 + magClearance      , one8+.05       , one4 + magClearance], center=true);
  }
}

// AA battery holder dimensions
/* The profile of the AA cell holder is 58mm x 26mm
  ** The O-ring of choice is 3/32" diameter (or .103" = 2.6mm) 
  ** The minimum Oring circumference, keeping it just outside the cell holder profile, is then 
  **  (58+5.2 + 29+5.2) * 2 = 188.8 mm, diameter = 60.1mm, = 2.366"
  ** Choose Dash Number 141, which is 2 1/2" OD. The circumference corresponding to the mean of the OD, ID is
  **     191.5 mm is the circumference of the oring
  ** To get to the actual profile, make the straight sides match the cell holder dimension:
  ** Remainder = 191.5 - (2 * (58 + 29)) = 17.5mm
  **  radius of a circle for that circumference is:  2.785mm
  */ 
  batt_holder_z_offset = 6;
  batt_holder_top_z_mm = 75;
  batt_holder_start_y_mm = 22; // will mirror to -y axis
  batt_holder_end_y_mm = 41;
  batt_holder_end_x_mm = 27;
  batt_holder_Ythick_mm = 5;
  batt_holder_Xthick_mm = 2;
  batt_holder_ywidth_mm = 10;
  batt_holder_brace_z_thick_mm = 14;
  oRingRadius = .103 * 25.4 * .5;
  cellFastenerDx = 6 * oRingRadius;
  AAcellW = 29;
  AAcellH = 58;
  oRingCellHolderRadius = 2.785;
  // 2 * (AAcellW + AAcellH) + 2 * pi * oRingCellHolderRadius = length of (i.e. circumference of) oring

  batt_holder_YZ_profile = [[batt_holder_start_y_mm, 0], 
                            [batt_holder_start_y_mm, batt_holder_top_z_mm],
                            [batt_holder_start_y_mm+batt_holder_ywidth_mm, batt_holder_top_z_mm],
                            [batt_holder_end_y_mm, batt_holder_brace_z_thick_mm],
                            [batt_holder_end_y_mm, 0]
                            ];

 
  batt_holder_XY_profile=[[batt_holder_end_x_mm, batt_holder_start_y_mm],
                          [batt_holder_end_x_mm, batt_holder_start_y_mm + batt_holder_Ythick_mm],
                          [-batt_holder_end_x_mm, batt_holder_start_y_mm + batt_holder_Ythick_mm],
                          [-batt_holder_end_x_mm, batt_holder_start_y_mm]];


/* number 2
  cellFastenNutHeight = .06 * 25.4;
  cellFastenBoltHoleDiameter = .09 * 25.4;
  cellFastenNutAcrossFlats = (3 / 16) * 25.4;
  cellFastenDriverClearanceDiameter = .13 * 25.4;
  cellFastenSocketHeadDiameter = .175 * 25.4;
  */

/* number 3 */
  cellFastenNutHeight = .07 * 25.4;
  cellFastenBoltHoleDiameter = .105 * 25.4;
  cellFastenNutAcrossFlats = .1865 * 25.4; // one thousandth's too small for tight fit
  cellFastenDriverClearanceDiameter = .115 * 25.4;
  cellFastenSocketHeadDiameter = .175 * 25.4;
/* */  

/* number 4 
  cellFastenNutHeight = .1 * 25.4;
  cellFastenBoltHoleDiameter = .12 * 25.4;
  cellFastenNutAcrossFlats = (1/4) * 25.4;
  cellFastenDriverClearanceDiameter = .145 * 25.4;
  cellFastenSocketHeadDiameter = .195 * 25.4;
  */

  minXFastener = -cellFastenerDx;
  minYFastener = -cellFastenerDx;
  maxYFastener = AAcellH + cellFastenerDx;
  maxXFastener = AAcellW + cellFastenerDx;

  numYfasteners = 4;

  // the bottom row of fasteners is omitted in favor of latching the bottom edge of the AA cover into the base
  fastenerHolePositionsLeft = [ for (y = [1:1:numYfasteners-1]) [minXFastener, minYFastener + y * ( AAcellH + 2 * cellFastenerDx) /(numYfasteners-1)]];
  fastenerHolePositionsRight = [ for (p = fastenerHolePositionsLeft) [maxXFastener, p[1]]];
  fastenerHolesLR = [
                    [0.5 * (fastenerHolePositionsLeft[0][0] + fastenerHolePositionsRight[0][0]), fastenerHolePositionsLeft[len(fastenerHolePositionsLeft)-1][1] ]];

  fastenerHolePositions = concat(fastenerHolePositionsLeft, fastenerHolePositionsRight, fastenerHolesLR);
  fastenerHoleOverlapmm = 2.7;
  facingOringXextent = maxXFastener-minXFastener + fastenerHoleOverlapmm*2;
  facingOringYextent = maxYFastener-minYFastener + fastenerHoleOverlapmm*2;
  facingOringRoundingRadiusmm = 2;
  thicknessAtScrewHeadsmm = 3;

  bodyHoleOverlapmm = -3.5;
  bodyXextent = maxXFastener-minXFastener + 2 * bodyHoleOverlapmm;
  bodyYextent = maxYFastener-minYFastener + 2 * bodyHoleOverlapmm;
  bodyThicknessmm = 19;

module positionAAcells(zOffset=batt_holder_z_offset)
{
  // 
  mirror([0,1,0])
  {
    translate([-AAcellW/2, batt_holder_start_y_mm + batt_holder_Ythick_mm, zOffset])
    rotate([90,0])  
      children();
  }
}

mainFastenerXY = [for(dx = [-1,1], dy = [-1, 1]) [33.5*dx, 25*dy]];

module mainFasteners()
{
  for(pos = mainFastenerXY)
          translate(pos) children();      
}

module base(){
  color("yellow")
  {
  difference()
  {
    union(){
      translate([0, 0, -base_disk_thickness/2])
      difference(){
        cylinder(d=base_diameter_mm, h=base_disk_thickness, center=true, $fn=fnval);
      
        //water holes
        for(dy = [-2:1:2], dx = [-1, 0, 1]){
          translate([-39*dx, 5*dy, 0]) cube([14, 4, base_disk_thickness], center = true);
        }  

        for(dy = [-1]){
          translate([0, 6*dy-35, 0]) cube([30, 4, base_disk_thickness], center = true);
        }    
        
        for(dy = [-1:1:1], dx = [-1, 1]){
          translate([10*dx, 6*dy+35, 0]) cube([10, 4, base_disk_thickness], center = true);
        }   
        
        //screw holes
        mainFasteners()
        {
          cylinder(d=base_mount_hole_diam_mm, h=base_disk_thickness, center=true, $fn=fn30val);
        }
        
        //cable hole
        translate([0, 31, 0]) cube([5, 8, base_disk_thickness], center = true);
      }
      // AA cell holder positive
      for (xpos = [batt_holder_end_x_mm,-batt_holder_end_x_mm])
        translate([xpos-batt_holder_Xthick_mm/2,0,0])
        mirror([0,1,0])
        translate([0,batt_holder_start_y_mm])
        rotate([0,90,0])
          linear_extrude(height=batt_holder_Xthick_mm)
            rotate([0,0,90])
              translate([-batt_holder_start_y_mm,0])
              polygon(batt_holder_YZ_profile);
      mirror([0,1,0])
          linear_extrude(height=batt_holder_top_z_mm)
            polygon(batt_holder_XY_profile);
    
      //thread  
      difference()
      {
        ScrewThread(base_funnel_thread_diameter_mm, base_funnel_thread_height_mm, base_funnel_thread_pitch_height_mm);
        cylinder(d=base_funnel_thread_diameter_mm - 10, h = base_funnel_thread_height_mm);
         // remove thread that blocks battery cell container extraction
        xExtent = batt_holder_end_x_mm *2 - batt_holder_Xthick_mm;
        bigEnoughXYZ = 100;
        zLipHeightmm = 7;
        // * removes this cut
        translate([-xExtent/2, -bigEnoughXYZ -batt_holder_start_y_mm-batt_holder_Xthick_mm, zLipHeightmm])
          cube([xExtent, bigEnoughXYZ, bigEnoughXYZ]);
     }          
    }
    positionAAcells()
    {
      // O ring slot
      linear_extrude(height= 2*oRingRadius * .9)
      {
        difference()
        {
          offset(oRingCellHolderRadius + oRingRadius)
            square([AAcellW, AAcellH]);
          offset(oRingCellHolderRadius - oRingRadius)
            square([AAcellW, AAcellH]);
        }
      }

      // fastener holes
      for (pos = fastenerHolePositions)
      translate(pos)
      {
        translate([0,0,  batt_holder_Ythick_mm-cellFastenNutHeight])
        linear_extrude(height = cellFastenNutHeight)
        {
          hex = 6;
          // convert across-flats to the corner distance from center
          c = cellFastenNutAcrossFlats * acrossFlatsConvert;
          profile = [for (i = [0:1: hex-1]) 
            let (angle = i * 360 / hex) 
              .5 * c * [cos(angle), sin(angle) ] ];
            polygon(profile);
        }
        linear_extrude(batt_holder_Ythick_mm)
          circle(d=cellFastenBoltHoleDiameter);
      }

      bodyAACoverExpand = .007 * 25.4;
      coverRotationAngleToInstall = -8;
      bodyAACover(expandMM = bodyAACoverExpand); // base clears AA cover body
      translate([0,-1])
        bodyAACover(expandMM = bodyAACoverExpand); // AA cover slides 1mm lower through base
      translate([0,0.5])  // AA cover tilted and slightly raised
        bodyAACover(expandMM = bodyAACoverExpand, coverRotate=coverRotationAngleToInstall);

        // wire hole
      wireHoleDiametermm = .075 * 25.4;
      translate([AAcellW/2, AAcellH ])
      {
        tiltForDrip = [0,-.2,1];
        tiltForDripV = tiltForDrip/norm(tiltForDrip);
        linear_extrude(height = batt_holder_Ythick_mm/tiltForDripV[2] + 0.1, v=tiltForDripV)
        {
           translate([wireHoleDiametermm,0])
            circle(d = wireHoleDiametermm);
          translate([- wireHoleDiametermm,0])
            circle(d = wireHoleDiametermm);
        }
      }
    }
  }
  
  //screw holes
  mainFasteners() {
    difference(){
      postHeightmm = 5;
      cylinder(d=12, h=postHeightmm, center=false, $fn=fn30val);
      cylinder(d=base_mount_hole_diam_mm, h=postHeightmm, center=false, $fn=fn30val);
      translate([0, 0, postHeightmm-number8MachineScrewSocketHeadHeight]) 
        cylinder(d=number8MachineScrewSocketHeadDiameter, h=number8MachineScrewSocketHeadHeight, center=false, $fn=fn30val);
    }
  }
  
  //cable holder
  if (cable_holder)
    translate([0, 31+8/2+10/2, 20/2]) cube([8, 10, 20], center = true);
  
  //border
  translate([0, 0, 16/2])
  difference(){
    cube([92+2*2, 24+2*2, 16], center = true);
    cube([92, 24, 16], center = true);
  }
  
  //holder
  difference(){
    translate([0, 0, 18/2+1/2]) cube([12, 20+2*2+2*5, 19], center = true);
    translate([0, 0, 18/2]) cube([12, bucket_slot_width_mm, 20], center = true);
    translate([0, 0, 18]) rotate([90, 0, 0]) cylinder(d=3.25, h=20+2*2+2*3, center=true, $fn=fn30val);
  }
  
  //bucket stops
  difference(){
      for(dx = [-1, 1]) {
        translate([22*dx, 0, 3]) scale([dx, 1, 1]) {
          translate([10/2, 0, -3/2]) cube([10, 24, 3], center = true);
          triangleRa(10, 20, 24, true);
        }
      }
      translate([0, 0, 10/2]) cube([70, 20-2*2, 10], center = true);
  }
  
  //pcb holder  
  translate([0, 19, 16/2])
  difference(){
    cube([50, 10, 16], center = true);
    cube([40, 4, 16], center = true);
  }
  }
}

// when printed on an FDM printer, the base needs some supports
module base_supports() {
    color("purple")
  positionAAcells()
  {
    linear_extrude(height= 2*oRingRadius )
    {
      difference()
          offset(4 + oRingRadius)
          square([AAcellW, AAcellH]);
      }
  }
}

module bodyAACover(expandMM = 0, coverRotate = 0)
{
  thicknessOfAngledTransition = 7;
  minY = minYFastener-fastenerHoleOverlapmm;
  translate([0,minY])
  rotate([coverRotate, 0,0])
  translate([0,-minY])
  mirror([0,0,1])
  {
    ehalf = expandMM/2;
    translate([minXFastener-fastenerHoleOverlapmm-ehalf, minY-ehalf])
    {
      // surface facing the oring
      linear_extrude(height=thicknessAtScrewHeadsmm + expandMM)
        offset(facingOringRoundingRadiusmm)
        offset(-facingOringRoundingRadiusmm)
          square([facingOringXextent+expandMM , facingOringYextent+expandMM]);
      translate([facingOringXextent/2+ehalf, facingOringYextent/2+ehalf, thicknessAtScrewHeadsmm])
      linear_extrude(height=thicknessOfAngledTransition+expandMM, 
              scale = .9 * [bodyXextent/facingOringXextent, bodyYextent/facingOringYextent]
          )
        offset(facingOringRoundingRadiusmm)
        offset(-facingOringRoundingRadiusmm)
          square([facingOringXextent+expandMM , facingOringYextent+expandMM], center=true);
    }
    translate([minXFastener-bodyHoleOverlapmm-ehalf, minYFastener-bodyHoleOverlapmm-ehalf])
    linear_extrude(height=bodyThicknessmm+expandMM)
      offset(facingOringRoundingRadiusmm)
      offset(-facingOringRoundingRadiusmm)
      square([ bodyXextent+expandMM , bodyYextent+expandMM]);
  }
}

module cellAACover()
{
  bodyInnerOverlapmm = -6.5;
  thicknessAtBackmm = 1.5;
  color("green")
  positionAAcells()
  {
    difference()
    {
      bodyAACover();
      mirror([0,0,1]) // hollow interior
      translate([minXFastener-bodyInnerOverlapmm, minYFastener-bodyInnerOverlapmm])
          linear_extrude(height=bodyThicknessmm-thicknessAtBackmm)
            offset(facingOringRoundingRadiusmm)
            offset(-facingOringRoundingRadiusmm)
            square([maxXFastener-minXFastener + 2 * bodyInnerOverlapmm  , maxYFastener-minYFastener + 2 * bodyInnerOverlapmm]);

      for (pos = fastenerHolePositions)
        translate(pos)
        {
          mirror([0,0,1])
            cylinder(h=bodyThicknessmm, d=cellFastenBoltHoleDiameter);
          mirror([0,0,1])
            translate([0,0,thicknessAtScrewHeadsmm])
              cylinder(h=bodyThicknessmm, d=cellFastenSocketHeadDiameter);
        }  
    }
  }
}

module funnel(){
  color("white")
  {
    dim = 10 * sqrt(100/PI); // 10000 sq mm
    echo("dim=", dim);
    difference(){
      union(){
        translate([0, 0, funnel_height_mm])
          mirror([0, 0, 1])
          rotate_extrude(angle = 360, convexity = 2, $fn=fnval)
            difference(){
              resize([dim+2, funnel_inner_height_mm+2])  polygon([[0,0], [1,0], [0,1]], paths=[[0,1,2]]);
              resize([dim, funnel_inner_height_mm])  polygon([[0,0], [1,0], [0,1]], paths=[[0,1,2]]);
            }
              
        difference(){
          cylinder(r=dim+2, h=funnel_height_mm, center=false, $fn=fnval);
          translate([0,0,-.05])
            cylinder(r=dim, h=funnel_height_mm+.1, center=false, $fn=fnval);
        }
        
        
        if (1) translate([0, 0, funnel_height_mm-funnel_inner_height_mm-10+2.5]) 
          cylinder(d=7, h=10, center=false, $fn=fn30val);
      }
    translate([0, 0, funnel_height_mm-funnel_inner_height_mm-10+2.5-.05]) 
      cylinder(d=5, h=10.1, center=false, $fn=fn30val);
  }
  
   //thread  

  dim1 = dim+1;
  thickXY = base_funnel_thread_diameter_mm/8;
  hei = base_funnel_thread_height_mm + thickXY;
  ScrewHole(base_funnel_thread_diameter_mm, hei, pitch=base_funnel_thread_pitch_height_mm)
    rotate_extrude()
    {
      poly= [[0,0], [thickXY,0], [thickXY,hei],[0,base_funnel_thread_height_mm]];
      translate([dim1-thickXY, 0])
        polygon(poly);
    }
  }
}

module sensor(){
    difference(){
        union(){
            translate([0, -0.25/2, 0]) cube([40-0.25, 4-0.25, 55], center = true);
            
            translate([0, -3, 12])
            difference(){
                cube([5, 3, 14], center = true);
                translate([0, -3/2+2.25/2, 0]) cylinder(d=2.25, h=8, center=true, $fn=fn30val);
                translate([0, -3/2+2.25/4, 0]) cube([2.25, 2.25/2, 8], center = true);
                translate([0, -3/2+2.25/4, 0]) cube([0.75, 2.25/2, 12], center = true);
            }
            
        }
                
        translate([0, -1, 12+6-0.5]) cube([1, 8, 1], center = true);
        translate([0, -1, 12-6+0.5]) cube([1, 8, 1], center = true);
        
        translate([0, 1.75/2, 12]) cube([2, 1.75, 20], center = true);
        
        for(dx = [-1, 1]) 
        translate([-10*dx, (3.75-3)/2, 12]) rotate([90, 0, 0]) cylinder(d=2.7, h=3, center=true, $fn=fn30val);
     }
}

module sensor_back(){
    difference(){
        translate([0, 0, 0]) cube([30, 3, 30], center = true);
    
        for(dx = [-1, 1]) 
            translate([-10*dx, 0, 0]) rotate([90, 0, 0]) cylinder(d=3.2, h=5, center=true, $fn=fn30val);

        hull(){
            translate([0, 0, 1]) rotate([90, 0, 0]) cylinder(d=2, h=5, center=true, $fn=fn30val);
            translate([0, 0, -1]) rotate([90, 0, 0]) cylinder(d=2, h=5, center=true, $fn=fn30val);
        }
    }
    
    for(dx = [-1, 1]) 
        translate([-10*dx, (3+2)/2, 0]) rotate([90, 0, 0])
            difference(){
               cylinder(d=10, h=2, center=true, $fn=fn30val);
               cylinder(d=6, h=2, center=true, $fn=fn30val);
            }
    
}

steel_post_beam_width_mm = 25.4 * (1 + 5/16 + 1/8)  ;
steel_post_thickness_mm = 25.4 * (.53);
steel_post_hanger_protrusion = 25.4 * .375;
steel_post_profile = [[-steel_post_beam_width_mm/2,0],
                      [ -steel_post_thickness_mm/2,0],
                      [ -steel_post_thickness_mm/2,-steel_post_hanger_protrusion],
                      [ steel_post_thickness_mm/2, -steel_post_hanger_protrusion],
                      [ steel_post_thickness_mm/2, 0],
                      [ steel_post_beam_width_mm/2,0],
                      [ steel_post_beam_width_mm/2,steel_post_thickness_mm],
                      [ steel_post_thickness_mm/2, steel_post_thickness_mm],
                      [ steel_post_thickness_mm/2, steel_post_beam_width_mm - 3],
                      [ -steel_post_thickness_mm/2, steel_post_beam_width_mm - 3],
                      [ -steel_post_thickness_mm/2, steel_post_thickness_mm],
                      [-steel_post_beam_width_mm/2, steel_post_thickness_mm],
];

module flange() {
    flangeCircleScale = 3;
    color ("blue")
    translate([0,0,flange_start_z_mm])
    mirror([0,0,1])
    difference()
    {
      union()
      {
        {
          for (pos = mainFastenerXY) 
            translate(pos)
            {
              cylinder(d=flange_fastener_diameter_mm, h=flange_fastener_height_mm);
              transV = [-pos[0],-pos[1], flange_transitionmm];
              transVNorm = transV / norm(transV); 
              translate([0,0,flange_fastener_height_mm])
              {
                linear_extrude(height=flange_transitionmm / transVNorm[2], v=transVNorm, scale=flangeCircleScale)
                  circle(d=flange_fastener_diameter_mm);
              }
            }
          translate([0,0,flange_fastener_height_mm+flange_transitionmm-.01])
          difference()
            {
              linear_extrude(height = flange_post_overlap)
                circle(d=flangeCircleScale * flange_fastener_diameter_mm);
              translate([0, -.345 * steel_post_beam_width_mm, -.1])
                linear_extrude(height = flange_post_cut_depth + .2)
                  polygon(steel_post_profile);
            }
        }
      }

      for (pos = mainFastenerXY)
        translate(pos)
        {
          //screw hole
          translate([0,0,-.01])
          cylinder(h=flange_transitionmm+flange_fastener_height_mm + .02, d = base_mount_hole_diam_mm);
          //hex nut
          translate([0,0,flange_fastener_height_mm*2])
          linear_extrude(height = flange_transitionmm)
          {
            hex = 6;
            // convert across-flats to the corner distance from center
            c = number8MachineNutAcrossFlats * acrossFlatsConvert;
            profile = [for (i = [0:1: hex-1]) 
              let (angle = i * 360 / hex) 
                .5 * c * [cos(angle), sin(angle) ] ];
              polygon(profile);
          }          
        }
    }
}

module assembled(){
    translate([0, 0, -18])
    {
      if (show_base) base();
      if (show_cellAACover) cellAACover();
      if (show_base_supports) base_supports();
    }
    if (show_flange) flange();
    if (show_bucket) rotate([0, 0, 0]) translate([0, 0, -3]) bucket();
    if (show_sensor) translate([0, 19, 55/2-18]) sensor();
    if (show_sensor_back) translate([0, 24, 55/2-18+12]) sensor_back();
    if (show_funnel) translate([0, 0, -18]) funnel();
}

module exploded(){
    if (show_base) translate([0, 0, -18])base();
    if (show_cellAACover) cellAACover();
    if (show_bucket) rotate([0, 0, 0]) translate([0, 0, 20]) bucket();
    if (show_sensor) translate([0, 19, 30]) sensor();
    if (show_sensor_back) translate([0, 30, 42]) sensor_back();
    if (show_funnel) translate([0, 0, 70]) funnel();
}

if (view_as_assembled != 0)
  {
    if (view_as_assembled == 1)
      assembled();
    else if (view_as_assembled == 2)
      intersection()
      {
        translate([0,100])
          cube([200, 200, 200], center=true);
        assembled();
      }
    else if (view_as_assembled == 3)
      intersection()
      {
        translate([100,0])
          cube([200, 200, 200], center=true);
        assembled();
      }
    else if (view_as_assembled == 4)
      { // orient for 3D print
        if (show_base) translate([0, 0, base_disk_thickness])base();
        translate([0, -140, 46])
        rotate([90,0])
        if (show_cellAACover) 
             cellAACover();
        if (show_bucket) translate([0, 100, 0]) bucket();
        if (show_funnel) translate([140, 0, funnel_height_mm]) 
          rotate([180,0])
            funnel();
        if (show_flange)
          rotate([180,0,0])
          translate([-150,0, -flange_start_z_mm])
            flange();
        if (show_sensor) translate([-140, 0, 0]) rotate([-90,0]) sensor();
        if (show_sensor_back) translate([0, -100, 0]) rotate([90,0]) sensor_back();
      }
    else if (view_as_assembled == 5) // intersect base & AA
    {
      // should be empty
        intersection() {
          base();
          cellAACover();
        }
    }
  }
else
  exploded();
