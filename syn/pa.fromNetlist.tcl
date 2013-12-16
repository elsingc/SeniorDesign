
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name Mojo-Base -dir "D:/MyDocs/Desktop/Mojo-Base/syn/planAhead_run_3" -part xc6slx9tqg144-2
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "D:/MyDocs/Desktop/Mojo-Base/syn/mojo_top.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {D:/MyDocs/Desktop/Mojo-Base/syn} }
set_property target_constrs_file "D:/MyDocs/Desktop/Mojo-Base/src/mojo.ucf" [current_fileset -constrset]
add_files [list {D:/MyDocs/Desktop/Mojo-Base/src/mojo.ucf}] -fileset [get_property constrset [current_run]]
link_design
