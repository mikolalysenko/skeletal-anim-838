#!/bin/bash
./motool -c data/EAStrafe/turns/1_RP_FIFA2005_APR_605_1.bvh > tmp/walk.mog

./motool -a data/EAStrafe/turns/1_RP_FIFA2005_APR_605_1.bvh tmp/walk.mog > tmp/walk7.mog
#./motool -a data/EAStrafe/turns/2_RP_FIFA2005_APR_605_2.bvh tmp/walk7.mog > tmp/walk8.mog
#./motool -a data/EAStrafe/turns/3_RP_FIFA2005_APR_605_3.bvh tmp/walk8.mog > tmp/walk9.mog  
#./motool -a data/EAStrafe/turns/4_RP_FIFA2005_APR_605_4.bvh tmp/walk9.mog > tmp/walk10.mog  
#./motool -a data/EAStrafe/turns/5_RP_FIFA2005_APR_605_5.bvh tmp/walk10.mog > tmp/walk11.mog
#./motool -a data/EAStrafe/turns/6_RP_FIFA2005_APR_606_1.bvh tmp/walk11.mog  > tmp/walk12.mog

./motool -scc tmp/walk7.mog > tmp/walk_final.mog

./motool -s 1000 tmp/walk_final.mog > test.bvh
./motool -path 10 data/test.spline tmp/walk_final.mog > test2.bvh

#./motool -a data/EAStrafe/straight/FABMGWK01-marking_walk_forward.bvh tmp/walk12.mog > tmp/walk2.mog
#./motool -a data/EAStrafe/straight/FABMGJOG1-marking_jog_forward.bvh  tmp/walk2.mog > tmp/walk3.mog
#./motool -a data/EAStrafe/straight/FABMGRUN1-marking_sprint_forward.bvh  tmp/walk3.mog > tmp/walk6.mog


#./motool -a data/EAStrafe/straight/FABMGJOG4-marking_jog_180.bvh
#./motool -a data/EAStrafe/straight/FABMGJOG2-marking_jog_45.bvh    
#./motool -a data/EAStrafe/straight/FABMGJOG5-marking_jog_135.bvh  
#./motool -a data/EAStrafe/straight/FABMGJOG3-marking_jog_90.bvh

#./motool -a data/EAStrafe/straight/FABMGRUN3-marking_sprint_90.bvh 
#./motool -a data/EAStrafe/straight/FABMGRUN2-marking_sprint_45.bvh  
#./motool -a data/EAStrafe/straight/FABMGRUN5-marking_sprint_135.bvh    
#./motool -a data/EAStrafe/straight/FABMGRUN4-marking_sprint_180.bvh  

#./motool -a data/EAStrafe/straight/FABMGWK02-marking_walk_45_right.bvh tmp/walk2.mog > tmp/walk3.mog
#./motool -a data/EAStrafe/straight/FABMGWK03-marking_walk_90_right.bvh tmp/walk3.mog > tmp/walk4.mog
#./motool -a data/EAStrafe/straight/FABMGWK04-marking_walk_180.bvh tmp/walk4.mog > tmp/walk5.mog
#./motool -a data/EAStrafe/straight/FABMGWK05-marking_walk_135_right.bvh tmp/walk5.mog > tmp/walk6.mog

