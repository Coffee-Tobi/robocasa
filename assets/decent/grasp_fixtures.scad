// RoboBarista: PARAMETRIC PROTOTYPE ONLY - dimension inputs must be measured.
// OpenSCAD: set part = "portafilter" or "pitcher"; export each half separately.
// Units millimeters. Never rely on friction-only grasp for hot liquid.
$fn=96;
part="portafilter"; // ["portafilter","pitcher"]
half="lower"; // ["lower","upper"]
// CALIBRATION REQUIRED: placeholder measurements, NOT Decent specifications.
pf_diameter=32; pf_length=65;
pitcher_handle_width=18; pitcher_handle_thickness=7; pitcher_length=34;
clearance=0.4; liner=1.0;
wall=6; bolt_d=4.4; bolt_offset=11;
grip_width=24; grip_depth=12;
module half_clamp(inner_width,inner_height,L,rounded=false) {
  outer_width=inner_width+2*liner+2*wall;
  outer_height=inner_height+2*liner+2*wall;
  difference() {
    union(){
      translate([0,0,outer_height/4])
        cube([outer_width,L,outer_height/2],center=true);
      for(y=[-L/2+bolt_offset,L/2-bolt_offset])
        translate([0,y,outer_height/4]) cube([outer_width+12,9,outer_height/2],center=true);
      // External IR40 contact datum: symmetric flat, repeatable approach
      translate([0,0,outer_height/2+grip_depth/2])
        cube([grip_width,min(L-4,40),grip_depth],center=true);
    }
    // Profile channel: open towards mating half
    if(rounded)
      translate([0,0,0]) rotate([90,0,0]) cylinder(d=inner_width+2*liner+clearance,h=L+2,center=true);
    else
      translate([0,0,0]) cube([inner_width+2*liner+clearance,L+2,inner_height+2*liner+clearance],center=true);
    // Bolt holes from mating face. Bolts need final access check.
    for (x=[-outer_width/2-3,outer_width/2+3],y=[-L/2+bolt_offset,L/2-bolt_offset])
      translate([x,y,outer_height/4]) cylinder(d=bolt_d,h=outer_height+grip_depth+4,center=true);
  }
}
module selected() {
  if(part=="portafilter") half_clamp(pf_diameter,pf_diameter,pf_length,true);
  else half_clamp(pitcher_handle_width,pitcher_handle_thickness,pitcher_length,false);
}
// Mirror around mating plane, shift for print layout
if(half=="upper") translate([0,0,0]) mirror([0,0,1]) selected();
else selected();
