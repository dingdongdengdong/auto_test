
"use strict";

let ScenarioLoad = require('./ScenarioLoad.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let RadarDetections = require('./RadarDetections.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let VelocityCmd = require('./VelocityCmd.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let ShipState = require('./ShipState.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let Transforms = require('./Transforms.js');
let CollisionData = require('./CollisionData.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let GhostMessage = require('./GhostMessage.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let CtrlCmd = require('./CtrlCmd.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let Obstacles = require('./Obstacles.js');
let ExternalForce = require('./ExternalForce.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let Obstacle = require('./Obstacle.js');
let Lamps = require('./Lamps.js');
let DillyCmd = require('./DillyCmd.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let RobotOutput = require('./RobotOutput.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let CMDConveyor = require('./CMDConveyor.js');
let SaveSensorData = require('./SaveSensorData.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let TrafficLight = require('./TrafficLight.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let SensorPosControl = require('./SensorPosControl.js');
let IntscnTL = require('./IntscnTL.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let PRStatus = require('./PRStatus.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let Conveyor = require('./Conveyor.js');
let RobotState = require('./RobotState.js');
let MapSpec = require('./MapSpec.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let ERP42Info = require('./ERP42Info.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let PREvent = require('./PREvent.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let IntersectionControl = require('./IntersectionControl.js');
let WaitForTick = require('./WaitForTick.js');
let VehicleSpec = require('./VehicleSpec.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let GPSMessage = require('./GPSMessage.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let SVADC = require('./SVADC.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let ObjectStatus = require('./ObjectStatus.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let GVStateCmd = require('./GVStateCmd.js');
let WheelControl = require('./WheelControl.js');
let RadarDetection = require('./RadarDetection.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let ReplayInfo = require('./ReplayInfo.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let EventInfo = require('./EventInfo.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let TOF = require('./TOF.js');

module.exports = {
  ScenarioLoad: ScenarioLoad,
  DdCtrlCmd: DdCtrlCmd,
  MoraiTLIndex: MoraiTLIndex,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  EgoVehicleStatus: EgoVehicleStatus,
  VehicleCollision: VehicleCollision,
  RadarDetections: RadarDetections,
  FaultStatusInfo: FaultStatusInfo,
  SkateboardStatus: SkateboardStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  VelocityCmd: VelocityCmd,
  SyncModeCmdResponse: SyncModeCmdResponse,
  ShipState: ShipState,
  VehicleCollisionData: VehicleCollisionData,
  ShipCtrlCmd: ShipCtrlCmd,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  PRCtrlCmd: PRCtrlCmd,
  Transforms: Transforms,
  CollisionData: CollisionData,
  SetTrafficLight: SetTrafficLight,
  GhostMessage: GhostMessage,
  IntersectionStatus: IntersectionStatus,
  CtrlCmd: CtrlCmd,
  ManipulatorControl: ManipulatorControl,
  Obstacles: Obstacles,
  ExternalForce: ExternalForce,
  GVDirectCmd: GVDirectCmd,
  Obstacle: Obstacle,
  Lamps: Lamps,
  DillyCmd: DillyCmd,
  FaultInjection_Tire: FaultInjection_Tire,
  RobotOutput: RobotOutput,
  SyncModeSetGear: SyncModeSetGear,
  ObjectStatusList: ObjectStatusList,
  CMDConveyor: CMDConveyor,
  SaveSensorData: SaveSensorData,
  ObjectStatusExtended: ObjectStatusExtended,
  VehicleSpecIndex: VehicleSpecIndex,
  TrafficLight: TrafficLight,
  NpcGhostCmd: NpcGhostCmd,
  MoraiSrvResponse: MoraiSrvResponse,
  SensorPosControl: SensorPosControl,
  IntscnTL: IntscnTL,
  GeoVector3Message: GeoVector3Message,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  PRStatus: PRStatus,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  FaultInjection_Controller: FaultInjection_Controller,
  Conveyor: Conveyor,
  RobotState: RobotState,
  MapSpec: MapSpec,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  ERP42Info: ERP42Info,
  FaultInjection_Response: FaultInjection_Response,
  PREvent: PREvent,
  MapSpecIndex: MapSpecIndex,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  SyncModeResultResponse: SyncModeResultResponse,
  DillyCmdResponse: DillyCmdResponse,
  IntersectionControl: IntersectionControl,
  WaitForTick: WaitForTick,
  VehicleSpec: VehicleSpec,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  SyncModeInfo: SyncModeInfo,
  WoowaDillyStatus: WoowaDillyStatus,
  GPSMessage: GPSMessage,
  MultiPlayEventResponse: MultiPlayEventResponse,
  SVADC: SVADC,
  MoraiSimProcHandle: MoraiSimProcHandle,
  MoraiTLInfo: MoraiTLInfo,
  ObjectStatus: ObjectStatus,
  MultiPlayEventRequest: MultiPlayEventRequest,
  SyncModeRemoveObject: SyncModeRemoveObject,
  FaultInjection_Sensor: FaultInjection_Sensor,
  GVStateCmd: GVStateCmd,
  WheelControl: WheelControl,
  RadarDetection: RadarDetection,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  NpcGhostInfo: NpcGhostInfo,
  ReplayInfo: ReplayInfo,
  MoraiSimProcStatus: MoraiSimProcStatus,
  MultiEgoSetting: MultiEgoSetting,
  SyncModeCmd: SyncModeCmd,
  WaitForTickResponse: WaitForTickResponse,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  EventInfo: EventInfo,
  SyncModeAddObject: SyncModeAddObject,
  TOF: TOF,
};
