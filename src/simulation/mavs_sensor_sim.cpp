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
#include <iostream>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <stdlib.h>

//mavs sensor simulation
#include <simulation/sensor_sim.h>

int main(int argc, char *argv[]){
  int myid = 0;
#ifdef USE_MPI  
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif
  
  if (argc<2){
    if(myid==0)std::cerr<<"No input file listed on command line arg 1"<<
		 std::endl;
    exit(1);
  }
  
  mavs::SensorSimulation sim;
  std::string infile(argv[1]);
  sim.Load(infile);

  if (!sim.IsValid()){
    if (myid==0)std::cerr<<"The simulation is not valid, exiting"<<std::endl;
    exit(1);
  }
#ifdef USE_MPI  
  MPI_Barrier(MPI_COMM_WORLD);
#endif
  sim.Run();
#ifdef USE_MPI  
  MPI_Finalize();
#endif  
  return 0;
}
