#include <OpenSim/OpenSim.h>
#include <regex>

std::vector<std::string> split(const std::string& input, const std::string& regex) {
  // passing -1 as the submatch index parameter performs splitting
  std::regex re(regex);
  std::sregex_token_iterator
    first{ input.begin(), input.end(), re, -1 },
    last;
  return{ first, last };
}

void parseImuFile(OpenSim::Storage& stor, OpenSim::Array<double>& time, OpenSim::Array<OpenSim::Array<OpenSim::Array<double> > > &outData, OpenSim::Array<std::string> &sensorNames) {
  stor.getTimeColumnWithStartTime(time);
  auto& labels = stor.getColumnLabels();
  int nSensors = (labels.getSize() - 1) / 4;
  for (int s = 0; s < nSensors; ++s) {
    OpenSim::Array<OpenSim::Array<double> > tmp{};
    for (int i = 0; i < 4; ++i) {
      OpenSim::Array<double>  dataTmp{};
      stor.getDataColumn(labels.get(1 + s * 4 + i), dataTmp);
      tmp.append(dataTmp);
    }
    outData.append(tmp);
    std::string label = labels.get(1 + s * 4);
    //std::string sensorName = split(label, "_").at(0);
    std::string sensorName = split(label, "-").at(0);
    sensorNames.append(sensorName);
  }
}

OpenSim::Array<double> prepareSensorDataFrame(OpenSim::Array<OpenSim::Array<double> >& data, double time, int index) {
  OpenSim::Array<double> res{};
  res.append(time);
  for (int i = 0; i < 4; ++i)
    res.append(data.get(i).get(index));
  return res;
}

SimTK::Rotation parseImuDataFrame(OpenSim::Array<double> sample, SimTK::Rotation earthToOsim) {
  SimTK::Quaternion quat(sample.get(1), sample.get(2), sample.get(3), sample.get(4));

  // std::cout << sample.get(0) << "   " << quat << std::endl;
  SimTK::Rotation sampleRot(quat);
  // std::cout << sampleRot.toMat33() << std::endl;
  // NOTE: do not transpose the rotation matrix!
  return earthToOsim*sampleRot;
}

int main(int argc, char* argv[]) {

  bool resampleStorage = true;
  bool saveModel = true;
  bool saveResampledStorage = true;
  // bool useProvidedModel = true;

  if (argc<4) {
    std::cout << "Please provide input .mot file" << std::endl;
    std::cout << "Please provide an output folder" << std::endl;
    std::cout << "Please provide a model file" << std::endl;
    return 0;
  }
  std::string storageFileName = argv[1];
  std::string outFolder = argv[2];
	// NEEDS TO BE FIXED TO RUN WITH OPENSIM 3.3
	std::string modelFileName;
  //if (argc == 4)
    modelFileName = argv[3];
  // else {
  // modelFileName = "blocks_model.osim";
  // useProvidedModel = false;
  // saveModel = true;
  //}

  OpenSim::Model model{};

  //if (useProvidedModel) {
    model = OpenSim::Model(modelFileName);
    model.setName(modelFileName.substr(0, modelFileName.length() - 5));
    model.setUseVisualizer(true);
  //}
	// NEEDS TO BE FIXED TO RUN WITH OPENSIM 3.3
  //else {
  //  model.setName(modelFileName.substr(0, modelFileName.length() - 5));
  //  model.setUseVisualizer(true);

  //  auto slab = new OpenSim::Body{ "slab",
  //    1,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Inertia{ 0 } };

  //  auto balljoint = new OpenSim::BallJoint{ "balljoint",
		//	model.getGroundBody(),
  //    SimTK::Vec3{ 0, 1, 0 },
  //    SimTK::Vec3{ 0 },
  //    *slab,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Vec3{ 0 } };

		//OpenSim::Body brick{};
  //  brick.setFrameName("slab");
  //  brick.set_half_lengths(SimTK::Vec3{ 0.25, 0.5, 0.05 });
  //  brick.setOpacity(0.1);
  //  slab->addGeometry(brick);

  //  auto slab2 = new OpenSim::Body{ "slab2",
  //    1,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Inertia{ 0 } };

  //  auto balljoint2 = new OpenSim::BallJoint{ "balljoint2",
  //    model.getGround(),
  //    SimTK::Vec3{ 0, 2, 0 },
  //    SimTK::Vec3{ 0 },
  //    *slab2,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Vec3{ 0 } };

  //  OpenSim::Brick brick2{};
  //  brick2.setFrameName("slab2");
  //  brick2.set_half_lengths(SimTK::Vec3{ 0.25, 0.5, 0.05 });
  //  brick2.setOpacity(0.3);
  //  slab2->addGeometry(brick2);

  //  auto slab3 = new OpenSim::Body{ "slab3",
  //    1,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Inertia{ 0 } };

  //  auto balljoint3 = new OpenSim::BallJoint{ "balljoint3",
  //    model.getGround(),
  //    SimTK::Vec3{ 0, 3, 0 },
  //    SimTK::Vec3{ 0,0, 0 },
  //    *slab3,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Vec3{ 0 } };

  //  OpenSim::Brick brick3{};
  //  brick3.setFrameName("slab3");
  //  brick3.set_half_lengths(SimTK::Vec3{ 0.25, 0.5, 0.05 });
  //  brick3.setOpacity(0.5);
  //  slab3->addGeometry(brick3);

  //  auto slab4 = new OpenSim::Body{ "slab4",
  //    1,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Inertia{ 0 } };

  //  auto balljoint4 = new OpenSim::BallJoint{ "balljoint4",
  //    model.getGround(),
  //    SimTK::Vec3{ 0, 4, 0 },
  //    SimTK::Vec3{ 0 },
  //    *slab4,
  //    SimTK::Vec3{ 0 },
  //    SimTK::Vec3{ 0 } };

  //  OpenSim::Brick brick4{};
  //  brick4.setFrameName("slab4");
  //  brick4.set_half_lengths(SimTK::Vec3{ 0.25, 0.5, 0.05 });
  //  brick4.setOpacity(0.9);
  //  slab4->addGeometry(brick4);

  //  SimTK::Transform sensorPose;
  //  OpenSim::Brick sensor{};
  //  OpenSim::PhysicalOffsetFrame* imuFrame = new OpenSim::PhysicalOffsetFrame{ "imu_sensor", *slab, sensorPose };
  //  sensor.set_half_lengths(SimTK::Vec3{ 0.1, 0.2, 0.01 });
  //  sensor.setName("1");
  //  imuFrame->addGeometry(sensor);

  //  OpenSim::Brick sensor2{};
  //  OpenSim::PhysicalOffsetFrame* imuFrame2 = new OpenSim::PhysicalOffsetFrame{ "imu_sensor2", *slab2, sensorPose };
  //  sensor2.set_half_lengths(SimTK::Vec3{ 0.1, 0.2, 0.01 });
  //  sensor2.setName("2");
  //  imuFrame2->addGeometry(sensor2);

  //  OpenSim::Brick sensor3{};
  //  OpenSim::PhysicalOffsetFrame* imuFrame3 = new OpenSim::PhysicalOffsetFrame{ "imu_sensor3", *slab3, sensorPose };
  //  sensor3.set_half_lengths(SimTK::Vec3{ 0.1, 0.2, 0.01 });
  //  sensor3.setName("3");
  //  imuFrame3->addGeometry(sensor3);

  //  OpenSim::Brick sensor4{};
  //  OpenSim::PhysicalOffsetFrame* imuFrame4 = new OpenSim::PhysicalOffsetFrame{ "imu_sensor4", *slab4, sensorPose };
  //  sensor4.set_half_lengths(SimTK::Vec3{ 0.1, 0.2, 0.01 });
  //  sensor4.setName("4");
  //  imuFrame4->addGeometry(sensor4);

  //  model.addFrame(imuFrame);
  //  model.addBody(slab);
  //  model.addJoint(balljoint);

  //  model.addFrame(imuFrame2);
  //  model.addBody(slab2);
  //  model.addJoint(balljoint2);

  //  model.addFrame(imuFrame3);
  //  model.addBody(slab3);
  //  model.addJoint(balljoint3);

  //  model.addFrame(imuFrame4);
  //  model.addBody(slab4);
  //  model.addJoint(balljoint4);
  //}
  if(saveModel)
    model.print(outFolder + "/" + model.getName() + "_OS4.osim");

  auto stor = OpenSim::Storage(storageFileName);
  stor.setName(storageFileName.substr(0, storageFileName.length() - 4));
  if (resampleStorage) {
    double resamplingDt = 0.007;
    stor.resample(resamplingDt, 4);
    stor.setName(storageFileName.substr(0, storageFileName.length() - 4) + "_resampledTo" + std::to_string(resamplingDt) + "sec");
    if(saveResampledStorage)
      stor.print(outFolder + "/" +stor.getName() +".mot");
  }

  OpenSim::Array<double> time{};
  OpenSim::Array<OpenSim::Array<OpenSim::Array<double> > > outData{};
  OpenSim::Array<std::string> sensorNames{};
  parseImuFile(stor, time, outData, sensorNames);
  std::cout << sensorNames << std::endl;

  auto& state = model.initSystem();

  SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
  viz.setBackgroundType(viz.GroundAndSky);
  viz.setShowSimTime(true);
  viz.drawFrameNow(state);

  // ----------------- Setup assembler
  SimTK::Assembler assembler(model.updMultibodySystem());
  SimTK::Transform sensorPose;
  SimTK::OrientationSensors* oSensorAssCond = new SimTK::OrientationSensors();
  std::vector<SimTK::OrientationSensors::OSensorIx> indexs{};

  //if (!useProvidedModel) {
		//indexs.push_back(oSensorAssCond->addOSensor("Imu_13", model.updBodySet().get("slab").getIndex(), sensorPose.R()));
		//indexs.push_back(oSensorAssCond->addOSensor("Imu_14", model.updBodySet().get("slab2").getIndex(), sensorPose.R()));
		//indexs.push_back(oSensorAssCond->addOSensor("Imu_15", model.updBodySet().get("slab3").getIndex(), sensorPose.R()));
		//indexs.push_back(oSensorAssCond->addOSensor("Imu_16", model.updBodySet().get("slab4").getIndex(), sensorPose.R()));
  //}
  //else {
		indexs.push_back(oSensorAssCond->addOSensor("Imu_13", model.updBodySet().get("imu_pelvis").getIndex(),sensorPose.R()));
		indexs.push_back(oSensorAssCond->addOSensor("Imu_14", model.updBodySet().get("imu_thigh").getIndex(), sensorPose.R()));
		indexs.push_back(oSensorAssCond->addOSensor("Imu_15", model.updBodySet().get("imu_leg").getIndex(), sensorPose.R()));
		indexs.push_back(oSensorAssCond->addOSensor("Imu_16", model.updBodySet().get("imu_foot").getIndex(), sensorPose.R()));
  //}
  //SimTK::Rotation osimToEarth(SimTK::Mat33{ 1, 0, 0, 0, 0, -1, 0, 1, 0 });
  SimTK::Rotation osimToEarth(SimTK::Mat33{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
  //SimTK::Rotation osimToEarth(SimTK::Mat33{ 0, 0, 1, 1, 0, 0, 0, 1, 0 });
  oSensorAssCond->defineObservationOrder(std::vector<std::string>{"Imu_13", "Imu_14", "Imu_15", "Imu_16"});
  //oSensorAssCond->defineObservationOrder(std::vector<std::string>{"SI-001627", "SI-001629", "SI-001631", "SI-002480"});

  int nSensors = oSensorAssCond->getNumOSensors();
  std::vector<int> sensorsToModelSensors{};
  std::vector<SimTK::OrientationSensors::OSensorIx> sensorIxMatch{};
  int matches = 0;
  for (int s = 0; s < nSensors; ++s) {
    SimTK::OrientationSensors::OSensorIx mx = indexs.at(s);
    int found = sensorNames.findIndex(oSensorAssCond->getOSensorName(mx));
    std::cout << oSensorAssCond->getOSensorName(mx) << std::endl;
    if (found != -1) {
      sensorsToModelSensors.push_back(found);
      std::cout << found << std::endl;
      sensorIxMatch.push_back(mx);
      matches++;
    }
  }

  assembler.setAccuracy(1e-9);
  assembler.adoptAssemblyGoal(oSensorAssCond);

  //Setup kinematic analysis to save simulation outputs
  OpenSim::Kinematics kinematicsAnalisys(&model);
  kinematicsAnalisys.setRecordAccelerations(true);

  for (int t = 0; t < time.size(); ++t) {
    SimTK::Array_<SimTK::Rotation> obs{};
    for (int m = 0; m < matches; ++m) {
      std::cout << "Time " << std::endl;
      auto tmp = outData.get(sensorsToModelSensors.at(m));
      auto data = prepareSensorDataFrame(tmp, time.get(t), t);
      obs.push_back(parseImuDataFrame(data, osimToEarth));
      //  if(t==0)
      //  oSensorAssCond->moveOneObservation(oSensorAssCond->getObservationIxForOSensor(sensorIxMatch.at(m)), parseImuDataFrame(data, osimToEarth.transpose()));
    }
    oSensorAssCond->moveAllObservations(obs);
    if (t == 0) {
      state.setTime(time.get(t));
      assembler.initialize(state);
      assembler.assemble(state);
      kinematicsAnalisys.begin(state);
    }
    if (t > 0) {
      oSensorAssCond->moveAllObservations(obs);
      assembler.initialize(state);
      assembler.assemble(state);
      state.setTime(time.get(t));
      assembler.track(state.getTime());
      assembler.updateFromInternalState(state);
      viz.drawFrameNow(state);
      kinematicsAnalisys.step(state, t);
    }
    std::cout << "Error at time t = " << time.get(t) << " " << assembler.calcCurrentGoal() << std::endl;
  }
  kinematicsAnalisys.end(state);
  kinematicsAnalisys.printResults(outFolder + "/" + stor.getName() + "_" + model.getName());
  return 0;
}
