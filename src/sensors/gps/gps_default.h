/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \file gps_default.h
 *
 * Default initialization data for the gps, based on ephemeris file 
 * "zeck001a.17n"
 *
 *\author Chris Goodin, cgoodin@cavs.msstate.edu
 *
 *\date 1/5/2018
 */

#ifndef GPS_DEFAULT_H
#define GPS_DEFAULT_H

namespace mavs{
namespace sensor{
namespace gps{

double gps_default[34][12] = {
  {-16708320.14, -3383960.786, 20328337.87, -941.4889239, -2420.773133,
   -1146.475362, 0.7048395105, -0.5084116484, 0.4946906713, 26530368.89,
   3874.816784, 590400},
  
  {-21328322.61, 14748143.8, 5239155.71, -703.7082976, -38.0706515,
   -3069.853815, -0.5410624177, -0.8301857953, 0.1343242551, 26454750.87,
   3880.350702, 7200},
  
  {6563085.42, 19565854.68, -16734779.07, -1809.514654, -1125.76706,
   -2025.711801, -0.7484972078, 0.557802387, 0.358620171, 26569711.89,
   3871.94691, 568800},
  
  {-17868974.78, -12037642.87, 15274408.17, -645.6322594, -1914.77685,
   -2183.930419, 0.7068312324, -0.6221917742, 0.336551638, 26410464.79,
   3883.6027, 590400},
  
  {-12788159.99, 9604431.88, 21143629.26, -2176.079225, -1611.568495,
   -561.9868939, 0.3911387555, -0.7255821534, 0.5661634151, 26511039.32,
   3876.229117, 7200},
  
  {-16107618.77, 10796525.34, 18165240.66, -2270.805663, -405.3604841,
   -1773.860931, -0.1524631014, -0.9030610064, 0.4015418053, 26570591.08,
   3871.882851, 0},
  
  {-6729415.323, 24996364.57, -4675653.718, -214.1299201, -656.7721927,
   -3135.823445, -0.9643552666, -0.2379787365, 0.1156937369, 26305227.81,
   3891.363338, 583200},
  
  {-20322146.32, -92780.20606, 17194433.02, -1657.393745, -1406.90416,
   -1966.943238, 0.3123010694, -0.8773246587, 0.364375473, 26620420.09,
   3868.257392, 583200},
  
  {-2433421.184, 16439573.52, -20745531.84, -2177.459975, -1478.361125,
   -917.9208652, -0.617608078, 0.5795371575, 0.5316925288, 26581162.64,
   3871.112833, 583200},
  
  {-10829039.97, 21125719.15, -11797785.71, -69.68970896, -1522.78932,
   -2669.301642, -0.9124874731, -0.3446386082, 0.2204333032, 26509467.44,
   3876.344036, 554400},
  
  {-21677825.07, -6340169.155, 13387972.09, -1030.926605, -1537.303989,
   -2304.370371, 0.4535310197, -0.8216537532, 0.3452458893, 26255735.4,
   3895.029246, 590400},
  
  {4508764.748, 19119485.52, -18116828.43, -1803.260877, -1361.624964,
   -1879.766007, -0.7716391866, 0.5238364376, 0.3607885147, 26722708.66,
   3860.846881, 7200},
  
  {4642293.998, 14340069.2, -21994543, -2638.008061, 810.1210725, -34.25666815,
   0.2354645992, 0.7906459044, 0.5651862316, 26663615.66, 3865.122797, 597600},
  
  {15590840.68, -21207927.23, 3101431.95, 453.6183214, -87.7021895, -3157.572709,
   0.7950006151, 0.5987086017, 0.0975809008, 26504138.75, 3876.733689, 0},
  
  {16074034.77, 8262230.476, -19728420.92, -2137.279538, 1249.357119,
   -1190.949569, 0.2014660954, 0.8341190208, 0.5134752882, 26755366.53,
   3858.489867, 597600},
  
  {-25615260.9, 156354.89, 7657159.57, -928.006073, -329.5949482, -3016.530356,
   0.02418921045, -0.9945691434, 0.1012279656, 26735708.9, 3859.908098, 576000},
  
  {-18266742.21, 15419133.55, 11737259.58, -1414.363528, 331.237386,
   -2735.144386,-0.5585525366, -0.8071598468, 0.1910812538, 26630561.63,
   3867.520761, 597600},
  
  {15106507.56, -21672759.42, -406555.885, 278.0088137, 330.7634185,
   -3108.296129,0.8142605986, 0.56506781, 0.1329588194, 26421210.41,
   3882.812881, 590400},
  
  {-17541618.54, 20142792.88, 708801.1687, -235.2862825, -50.53089508,
   -3186.989045, -0.7513330552, -0.6566265973, 0.06587982835, 26719709.67,
   3861.063543, 0},
  
  {5078085.04, 18770846.7, 17930171.81, -2098.138404, -1014.02096, 1642.383764,
   0.649937582, -0.6094862143, 0.4539908524, 26450381.75, 3880.67117, 7200},
  
  {17200824.22, 7650079.514, 19574314.49, -2041.146084, 1122.907922, 1393.272536,
   -0.1535838589, -0.8671106201, 0.473847202, 27157795.53, 3829.795278, 7184},
  
  {724069.8554, 18093821.6, -19166952.28, -2160.776757, -1271.782088,
   -1275.745431, -0.6398019671, 0.5707780538, 0.5146511985, 26368214.14,
   3886.712866, 568800},
  
  {-3230278.384, 20864262.1, -15629730.16, -1057.533748, -1770.714853, -2159.573,
   -0.9272075893, 0.1217807448, 0.3541970306, 26268623.8, 3894.073604, 575984},
  
  {18035756.34, 12521062.59, -15179844.19, -1920.466965, 47.07147689,
   -2230.323707, -0.3463259927, 0.8829871759, 0.3168468937, 26692567.97,
   3863.02606, 0},
  
  {16633715.9, 19185097.65, -8275421.421, -939.5737023, -453.4391775,
   -2966.125803, -0.7223638702, 0.6801500146, 0.1248454912, 26706386.43,
   3862.026524, 7200},
  
  {20279481.69, -255843.8017, 17194503.75, 1669.782469, 1411.124511,
   -1942.206676,-0.305663323, 0.8758087027, 0.3735358742, 26588978.79,
   3870.543812, 7184},
  
  {-23134116.78, -11495592.73, 6553126.092, -541.6069763, -666.9587115,
   -3045.778509, 0.4669596822, -0.8775198483, 0.1091218172, 26651068.89,
   3866.032499, 583200},
  
  {-12692153.2, 18476713.57, -13594215.51, 180.4614924, -1829.966683, -
   2539.100055, -0.873662071, -0.4220364214, 0.2420740481, 26216071.27,
   3897.974664, 597600},
  
  {5826263.074, 21856944.84, 13946174.39, -459.7446127, 1713.172995,
   -2490.781832,-0.9640027474, 0.09968563853, 0.246498431, 26573805.89,
   3871.64864, 7200},
  
  {-2789967.165, 22202733.84, -14222961.91, -846.7643599, -1621.546057,
   -2376.650472, -0.9535912751, 0.06806584165, 0.2933099405, 26514862.86,
   3875.949624, 590400},
  
  {25449464.22, -6210281, -5420273.797, -560.7096114, 458.7259637, -3070.054937,
   0.2554149334, 0.9619447842, 0.09708472529, 26751115.62, 3858.796423, 7200},
  
  {18792270.3, -17435193.73, -6887921.664, -295.0315014, 870.8344638,
   -3008.414079, 0.6999947901, 0.701390726, 0.1343813359, 26543904.54,
   3873.828706, 0},
  
  {13168851.19, -7513768.316, 21750120.33, 2095.266388, 1817.125339,
   -599.3569996, -0.4655269369, 0.710736287, 0.527388473, 26513073.95,
   3876.080382, 7200},
  
  {21222480, 16109721.16, 2001865.782, -363.4733128, 202.1511819, 3066.341449,
   0.5926805564, -0.7960315874, 0.1227333282, 26719360.77, 3861.088752, 7200}
};

} //namespace gps
} //namespace sensor
} //namespace mavs
 
#endif
