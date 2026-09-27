**Volume 13. Mapping Localization and SLAM**


# Chapter 12. Industrial SLAM Case Studies

##  

## 12.01. Indoor AMR Fleet SLAM Toolbox Production Case

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

A production indoor AMR fleet requires localization and mapping to operate reliably across warehouses, factories, hospitals, and logistics centers where layouts evolve continuously. A SLAM Toolbox-based architecture can provide the mapping and localization foundation while integrating with ROS 2 navigation, fleet management, safety controllers, and facility-level operational systems.

During initial deployment, one AMR can perform a controlled mapping mission through the facility while SLAM Toolbox processes laser scans, odometry, and transform information. Wheel encoders provide short-term motion estimates, while a 2D LiDAR supplies geometric observations of walls, columns, racks, and other persistent structures. The resulting pose graph represents both robot trajectory and spatial constraints.

Loop closure is particularly important in large industrial environments. As the mapping AMR returns to previously observed locations, SLAM Toolbox identifies compatible observations and introduces additional constraints into the pose graph. Graph optimization then distributes accumulated localization error across the trajectory, reducing drift that would otherwise become significant after long movements through corridors or repeated warehouse aisles.

Production mapping should distinguish permanent infrastructure from temporary objects. Walls, structural columns, fixed machinery, and stable rack boundaries are valuable localization references, whereas pallets, carts, workers, forklifts, and temporary inventory should not dominate the map. Mapping runs are therefore normally performed under controlled conditions, followed by engineering review before a map is released for fleet operation.

The approved occupancy map becomes a controlled production asset rather than merely an output of a SLAM experiment. Map resolution, coordinate origin, frame relationships, restricted regions, docking positions, charging stations, traffic zones, and navigation annotations should be versioned together. This allows the fleet management system to associate every operational robot with an explicitly approved map configuration.

Once the production map is established, AMRs normally operate primarily in localization mode instead of continuously modifying the shared reference map. Each robot compares current LiDAR observations with the established environment while odometry predicts short-term motion. This separation between controlled mapping and routine localization prevents temporary environmental changes from unintentionally corrupting the fleet\'s common spatial reference.

ROS 2 provides the integration layer between SLAM Toolbox and the remaining autonomy stack. The transform hierarchy typically connects the map, odom, and base frames, while sensor-specific transforms describe the positions of LiDARs and other devices. Navigation2 can consume the resulting robot pose and occupancy information to perform global planning, local obstacle avoidance, recovery behaviors, and goal execution.

A fleet introduces requirements beyond those of a single AMR. Every robot must interpret the same production coordinate system consistently so that fleet-level destinations correspond to identical physical locations. A command such as moving to Station A must therefore resolve to the same map location regardless of which AMR receives the task. Map distribution and configuration control consequently become fleet-management functions.

Localization quality should also be treated as an observable production variable. Pose uncertainty, scan matching quality, transform timing, odometry consistency, localization recovery events, and repeated navigation deviations can be collected as operational telemetry. A robot that repeatedly loses localization can then be identified before the problem develops into navigation failure, excessive recovery behavior, or mission interruption.

Industrial environments create difficult cases that must be considered during validation. Long feature-poor corridors, repetitive rack aisles, large open areas, reflective surfaces, glass partitions, and moving equipment can reduce geometric uniqueness. Wheel slip can further degrade odometry. Reliable deployment therefore depends on suitable LiDAR placement, accurate extrinsic calibration, robust odometry, and sufficient persistent environmental features.

Map changes require a controlled lifecycle. A moved pallet should normally be handled as a dynamic obstacle, while relocation of racks, installation of machinery, or structural modification may justify map revision. Engineers can perform a new mapping session or update an existing pose graph, validate navigation-critical regions, assign a new map version, and release it only after localization and route tests have passed.

Rolling out a new map to the fleet should resemble a software deployment rather than an uncontrolled file replacement. The fleet can retain the previous validated map, deploy the candidate version to selected AMRs, execute regression routes, verify docking and charging positions, and compare localization performance. If abnormalities appear, the system should support rapid rollback to the previously approved map and configuration.

Docking operations demand greater precision than ordinary corridor navigation. A global SLAM pose can guide an AMR into the docking region, but final alignment may use LiDAR geometry, fiducial markers, vision, reflectors, or dedicated docking sensors. This layered approach prevents the fleet from demanding unrealistic millimeter-level accuracy from the facility-wide SLAM system while still supporting repeatable charging and material-transfer operations.

Fleet-scale operation also requires clear separation between localization, navigation, traffic coordination, and mission orchestration. SLAM Toolbox determines the robot\'s relationship to the map, Navigation2 determines how the individual robot reaches a goal, and the fleet manager coordinates shared resources and prevents conflicting traffic. Higher-level orchestration assigns missions according to production priorities, robot state, and facility requirements.

Failure recovery must be designed before deployment. If localization confidence becomes unacceptable, the AMR should avoid continuing normal autonomous motion based on an unreliable pose. Depending on system design, it may stop safely, request relocalization, use a known recovery area, obtain an externally supplied initial pose, or request operator assistance. The fleet manager should recognize this state and redistribute pending missions when necessary.

Production acceptance should therefore evaluate complete operational scenarios rather than only mapping accuracy. Tests should include repeated routes, long-duration operation, localization after reboot, initialization at multiple positions, dynamic obstacle exposure, temporary aisle blockage, docking, map-server restart, network interruption, and recovery from localization loss. Performance should be evaluated statistically across repeated missions and multiple robots.

A mature deployment ultimately treats SLAM as shared infrastructure for the AMR fleet. The essential production capability is not simply generating an occupancy grid, but maintaining a trustworthy relationship among physical space, map versions, robot localization, navigation goals, and fleet-level operations. SLAM Toolbox can serve as the mapping and localization component within this broader architecture when supported by disciplined calibration, validation, monitoring, and map governance.

생산 환경의 실내 자율이동로봇(AMR, Autonomous Mobile Robot) 플릿(Fleet)은 창고, 공장, 병원, 물류센터와 같이 공간 구성이 지속적으로 변화하는 환경에서 안정적으로 동작하기 위해 위치추정(Localization)과 지도작성(Mapping) 기능을 필요로 한다. SLAM 툴박스(SLAM Toolbox) 기반 아키텍처(Architecture)는 ROS 2 내비게이션(Navigation), 플릿 관리(Fleet Management), 안전 제어기(Safety Controller), 시설 운영 시스템과 통합되면서 지도작성과 위치추정의 기반을 제공할 수 있다.

초기 구축(Initial Deployment) 단계에서는 하나의 AMR이 시설 내부에서 통제된 지도작성 임무(Mapping Mission)를 수행하고, SLAM 툴박스(SLAM Toolbox)가 레이저 스캔(Laser Scan), 오도메트리(Odometry), 좌표 변환(Transform) 정보를 처리할 수 있다. 휠 인코더(Wheel Encoder)는 단기적인 이동량을 추정하고, 2D 라이다(2D LiDAR)는 벽, 기둥, 랙(Rack)과 같은 지속적인 구조물의 기하학적 정보를 제공한다. 생성된 포즈 그래프(Pose Graph)는 로봇의 이동 궤적과 공간적 제약조건(Spatial Constraint)을 함께 표현한다.

루프 폐쇄(Loop Closure)는 특히 대규모 산업 환경에서 중요하다. 지도작성 AMR이 이전에 관측했던 위치로 다시 돌아오면 SLAM 툴박스(SLAM Toolbox)는 서로 일치할 수 있는 관측 정보를 식별하고 포즈 그래프(Pose Graph)에 추가적인 제약조건을 생성한다. 이후 그래프 최적화(Graph Optimization)는 누적된 위치추정 오차를 전체 이동 궤적에 분산시켜 긴 복도나 반복적인 창고 통로를 장시간 이동하면서 발생할 수 있는 드리프트(Drift)를 감소시킨다.

생산용 지도작성(Production Mapping)에서는 영구적인 시설 구조와 일시적인 객체를 구분해야 한다. 벽, 구조 기둥, 고정 설비, 안정적으로 배치된 랙 경계는 유용한 위치추정 기준이지만, 팔레트(Pallet), 카트(Cart), 작업자, 지게차, 임시 재고는 지도에 지배적으로 반영되어서는 안 된다. 따라서 지도작성 작업은 일반적으로 통제된 환경에서 수행하며, 생성된 지도는 플릿 운영에 배포되기 전에 엔지니어링 검토(Engineering Review)를 거치는 것이 바람직하다.

승인된 점유 지도(Occupancy Map)는 단순한 SLAM 실험의 결과물이 아니라 통제되는 생산 자산(Production Asset)이 된다. 지도 해상도(Map Resolution), 좌표 원점(Coordinate Origin), 프레임 관계(Frame Relationship), 제한 구역(Restricted Region), 도킹 위치(Docking Position), 충전소(Charging Station), 교통 구역(Traffic Zone), 내비게이션 주석(Navigation Annotation)을 함께 버전 관리해야 한다. 이를 통해 플릿 관리 시스템(Fleet Management System)은 모든 운영 로봇을 명확하게 승인된 지도 구성(Map Configuration)과 연결할 수 있다.

생산용 지도가 구축된 이후 AMR은 일반적으로 공유 기준 지도를 지속적으로 수정하는 대신 주로 위치추정 모드(Localization Mode)로 동작한다. 각 로봇은 현재의 라이다(LiDAR) 관측 결과를 기존 환경 지도와 비교하고, 오도메트리(Odometry)를 이용해 단기적인 움직임을 예측한다. 통제된 지도작성과 일상적인 위치추정을 분리하면 일시적인 환경 변화가 플릿 전체에서 사용하는 공통 공간 기준(Common Spatial Reference)을 의도하지 않게 손상시키는 것을 방지할 수 있다.

ROS 2는 SLAM 툴박스(SLAM Toolbox)와 나머지 자율주행 스택(Autonomy Stack)을 연결하는 통합 계층(Integration Layer)을 제공한다. 좌표 변환 계층(Transform Hierarchy)은 일반적으로 맵(Map), 오돔(Odom), 베이스(Base) 프레임을 연결하며, 센서별 좌표 변환은 라이다와 다른 장치의 위치를 정의한다. 내비게이션2(Navigation2)는 이렇게 생성된 로봇 위치와 점유 정보를 이용하여 전역 경로 계획(Global Planning), 지역 장애물 회피(Local Obstacle Avoidance), 복구 동작(Recovery Behavior), 목표 실행(Goal Execution)을 수행할 수 있다.

플릿(Fleet)은 단일 AMR보다 더 높은 수준의 요구사항을 가진다. 모든 로봇은 동일한 생산 좌표계(Production Coordinate System)를 일관되게 해석해야 하며, 플릿 수준에서 정의된 목적지는 어느 로봇에서도 동일한 물리적 위치를 의미해야 한다. 따라서 특정 로봇에 관계없이 'Station A로 이동'과 같은 명령은 동일한 지도상의 위치로 변환되어야 한다. 이에 따라 지도 배포(Map Distribution)와 구성 관리(Configuration Control)는 플릿 관리의 핵심 기능이 된다.

위치추정 품질(Localization Quality) 역시 관찰 가능한 생산 변수(Production Variable)로 관리해야 한다. 포즈 불확실성(Pose Uncertainty), 스캔 정합 품질(Scan Matching Quality), 좌표 변환 타이밍(Transform Timing), 오도메트리 일관성(Odometry Consistency), 위치추정 복구 이벤트(Localization Recovery Event), 반복적인 주행 편차 등을 운영 텔레메트리(Operational Telemetry)로 수집할 수 있다. 이를 통해 반복적으로 위치를 잃는 로봇을 실제 주행 실패나 임무 중단으로 발전하기 전에 식별할 수 있다.

산업 환경에는 검증 과정에서 반드시 고려해야 하는 어려운 조건들이 존재한다. 특징이 부족한 긴 복도, 반복적인 랙 통로, 넓은 개방 공간, 반사 표면, 유리 칸막이, 이동 장비 등은 기하학적 고유성(Geometric Uniqueness)을 감소시킬 수 있다. 휠 슬립(Wheel Slip)은 오도메트리 성능을 더욱 저하시킬 수 있다. 따라서 신뢰성 높은 구축을 위해서는 적절한 라이다 배치, 정확한 외부 파라미터 보정(Extrinsic Calibration), 강건한 오도메트리(Robust Odometry), 충분한 영구적 환경 특징(Persistent Environmental Feature)이 필요하다.

지도 변경(Map Change)은 통제된 수명주기(Lifecycle)를 통해 관리해야 한다. 이동된 팔레트는 일반적으로 동적 장애물(Dynamic Obstacle)로 처리할 수 있지만, 랙 재배치, 신규 설비 설치, 건축 구조 변경 등은 지도 개정(Map Revision)이 필요할 수 있다. 엔지니어는 새로운 지도작성 세션을 수행하거나 기존 포즈 그래프(Pose Graph)를 업데이트하고, 내비게이션 핵심 영역을 검증한 뒤 새로운 지도 버전(Map Version)을 부여하여 위치추정과 경로 시험을 통과한 경우에만 운영 환경에 배포할 수 있다.

새로운 지도를 플릿에 배포하는 과정은 통제되지 않은 파일 교체가 아니라 소프트웨어 배포(Software Deployment)와 유사하게 관리해야 한다. 플릿은 기존에 검증된 지도를 유지하면서 후보 버전(Candidate Version)을 일부 AMR에 먼저 적용하고, 회귀 경로 시험(Regression Route Test)을 수행하며, 도킹 및 충전 위치와 위치추정 성능을 검증할 수 있다. 이상이 발견되면 이전에 승인된 지도와 구성으로 신속하게 롤백(Rollback)할 수 있어야 한다.

도킹 작업(Docking Operation)은 일반적인 복도 주행보다 높은 정밀도를 요구한다. 전역 SLAM 포즈(Global SLAM Pose)는 AMR을 도킹 영역까지 유도할 수 있지만, 최종 정렬(Final Alignment)에서는 라이다 기하정보(LiDAR Geometry), 기준 마커(Fiducial Marker), 비전(Vision), 반사체(Reflector), 전용 도킹 센서(Docking Sensor)를 사용할 수 있다. 이러한 계층적 접근법(Layered Approach)은 시설 전체 SLAM 시스템에 비현실적인 밀리미터 수준의 정확도를 요구하지 않으면서도 반복 가능한 충전 및 자재 이송 작업을 지원한다.

플릿 규모의 운영에서는 위치추정(Localization), 내비게이션(Navigation), 교통 조정(Traffic Coordination), 임무 오케스트레이션(Mission Orchestration)의 역할을 명확하게 분리해야 한다. SLAM 툴박스(SLAM Toolbox)는 지도에 대한 로봇의 위치 관계를 결정하고, 내비게이션2(Navigation2)는 개별 로봇이 목표 위치까지 이동하는 방법을 결정한다. 플릿 관리자(Fleet Manager)는 공유 자원을 조정하고 교통 충돌을 방지하며, 상위 오케스트레이션 계층은 생산 우선순위, 로봇 상태, 시설 요구사항에 따라 임무를 할당한다.

장애 복구(Failure Recovery)는 실제 배포 전에 설계되어야 한다. 위치추정 신뢰도(Localization Confidence)가 허용 가능한 수준보다 낮아지면 AMR은 신뢰할 수 없는 위치 정보를 기반으로 정상 자율주행을 계속해서는 안 된다. 시스템 설계에 따라 안전 정지(Safe Stop), 재위치추정(Relocalization), 지정된 복구 영역(Recovery Area) 사용, 외부 초기 포즈(Initial Pose) 입력 또는 작업자 지원 요청 등의 방법을 사용할 수 있다. 플릿 관리자는 이러한 상태를 인식하고 필요하면 대기 중인 임무를 다른 로봇에 재분배해야 한다.

따라서 생산 승인(Production Acceptance)은 단순한 지도 정확도뿐만 아니라 완전한 운영 시나리오(Operational Scenario)를 평가해야 한다. 반복 경로, 장시간 운전, 재부팅 이후 위치추정, 다양한 위치에서의 초기화, 동적 장애물 노출, 임시 통로 차단, 도킹, 지도 서버(Map Server) 재시작, 네트워크 단절, 위치추정 손실 이후 복구 등을 시험해야 한다. 성능은 여러 로봇과 반복적인 임무 수행 결과를 기반으로 통계적으로 평가하는 것이 중요하다.

성숙한 구축 환경에서는 궁극적으로 SLAM을 AMR 플릿을 위한 공유 인프라(Shared Infrastructure)로 취급해야 한다. 핵심적인 생산 역량은 단순히 점유 격자(Occupancy Grid)를 생성하는 것이 아니라 물리적 공간, 지도 버전, 로봇 위치추정, 내비게이션 목표, 플릿 수준 운영 사이의 신뢰할 수 있는 관계를 지속적으로 유지하는 것이다. SLAM 툴박스(SLAM Toolbox)는 체계적인 보정(Calibration), 검증(Validation), 모니터링(Monitoring), 지도 거버넌스(Map Governance)가 함께 적용될 때 이러한 전체 아키텍처의 지도작성 및 위치추정 구성요소로 활용될 수 있다.

##  

## 12.02. Outdoor AMR LIO SAM Large Campus Mapping Case

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

Outdoor autonomous mobile robots operating across large campuses require localization that remains stable over long distances, uneven terrain, changing illumination, vegetation, buildings, and partially structured roads. LIO-SAM provides a practical foundation by tightly combining LiDAR and inertial measurements within a factor-graph framework, allowing an AMR to estimate motion while constructing a geometrically consistent three-dimensional map.

A typical campus mapping platform carries a 3D LiDAR, IMU, GNSS receiver, wheel odometry, and an onboard computing unit. The LiDAR captures surrounding geometry, while the IMU measures high-frequency angular velocity and linear acceleration. GNSS provides global position references when satellite visibility is sufficient, and wheel odometry can supply complementary vehicle-motion information, particularly during relatively stable ground contact.

Accurate sensor calibration is fundamental before mapping begins. The rigid-body transformation between LiDAR and IMU must be determined precisely, and their timestamps must be synchronized sufficiently for motion compensation. Incorrect extrinsic parameters or timing offsets can produce distorted point clouds, inconsistent scan alignment, and systematic trajectory errors that become increasingly visible as the AMR travels across a large campus.

LIO-SAM uses IMU measurements to estimate short-term motion and compensate for LiDAR motion distortion during each scan. This process is especially important for outdoor AMRs because vibration, acceleration, steering, and uneven terrain continuously change the sensor pose. Deskewed point clouds provide more reliable geometric observations for subsequent feature extraction, scan registration, and factor-graph optimization.

LiDAR observations contribute geometric constraints between robot poses. Edge-like and planar structures from buildings, curbs, walls, poles, road surfaces, and other persistent objects can support scan matching. Unlike indoor environments, however, outdoor campuses may contain vegetation, parked vehicles, pedestrians, delivery trucks, and construction equipment. Mapping quality therefore depends on emphasizing stable geometric structures while reducing dependence on transient objects.

The factor graph is the central estimation structure of LIO-SAM. Robot states are represented as graph variables, while IMU preintegration, LiDAR odometry, loop closures, and available GNSS observations introduce constraints between those states. Optimization combines these sources to estimate a trajectory that is more globally consistent than one obtained from continuously integrating local motion estimates alone.

Large campuses make accumulated drift particularly important. Even accurate LiDAR-inertial odometry gradually develops error over hundreds or thousands of meters. When the AMR revisits a previously mapped region, loop-closure detection can identify the correspondence and add a long-range constraint. Graph optimization then redistributes accumulated error through the trajectory and improves consistency between different sections of the campus map.

GNSS can complement loop closure by anchoring the locally accurate SLAM trajectory to a global geographic reference. Standard GNSS may provide useful coarse constraints, while RTK-GNSS can provide substantially higher positioning accuracy under suitable satellite and correction conditions. Global measurements should nevertheless be incorporated according to their uncertainty because multipath, buildings, trees, tunnels, and urban canyons can temporarily degrade GNSS reliability.

A useful production architecture therefore avoids treating GNSS as an unquestionable ground truth. LIO-SAM can combine relative LiDAR-inertial accuracy with appropriately weighted global observations, allowing the estimator to benefit from GNSS without blindly following corrupted measurements. Monitoring GNSS covariance, fix status, satellite geometry, and correction availability helps determine when global measurements should strongly or weakly influence the optimized trajectory.

Campus-scale mapping also requires careful coordinate management. The SLAM map commonly operates in a local Cartesian coordinate system, while GNSS observations originate from a geographic reference system. A defined transformation between geographic coordinates and a local frame such as East-North-Up enables maps, missions, infrastructure positions, and fleet destinations to share a consistent spatial reference without forcing every local computation to operate directly in latitude and longitude.

Terrain variation creates additional challenges for outdoor AMRs. Slopes, speed bumps, curbs, drainage channels, gravel, grass, and damaged pavement generate roll, pitch, vertical motion, and wheel slip that are largely absent from ideal planar navigation models. Three-dimensional LiDAR-inertial estimation preserves six-degree-of-freedom vehicle motion, making it more appropriate for such environments than localization architectures that assume a perfectly flat operating surface.

A production mapping mission should cover important roads and operational zones from multiple viewpoints rather than simply driving each path once. Revisiting intersections, building entrances, charging areas, narrow passages, and major loops creates redundant geometric constraints. These repeated observations strengthen loop closure and provide opportunities to identify map regions where localization quality is weak because of insufficient or repetitive environmental features.

Map density must be balanced against computational and storage requirements. Raw LiDAR data collected over a large campus can become extremely large, while autonomous navigation rarely requires every measured point. Voxel filtering, keyframe selection, local submaps, and optimized point-cloud representations can reduce data volume while preserving structures required for localization, obstacle interpretation, and engineering inspection.

The resulting three-dimensional map should be treated as a production asset with explicit versions and validation status. Buildings, roads, curbs, poles, fixed infrastructure, docking areas, charging stations, restricted regions, and mission landmarks can be associated with the geometric map. Changes caused by construction, landscaping, relocated equipment, or new buildings should trigger controlled assessment rather than automatic modification of the operational reference map.

Localization during routine operation can reuse the validated map instead of continuously rebuilding the entire campus. Current LiDAR and IMU measurements estimate local motion and align the AMR with the established environment, while GNSS provides global assistance where appropriate. This architecture reduces unnecessary map changes and allows multiple AMRs to operate against the same spatial reference while maintaining independent state estimates.

Operational monitoring should track more than whether the SLAM software is running. LiDAR registration quality, IMU bias, estimated covariance, GNSS residuals, loop-closure events, map matching performance, trajectory discontinuities, processing latency, and localization recovery frequency provide valuable indicators of system health. Long-term statistics can reveal deteriorating sensors, calibration shifts, environmental changes, or specific campus regions that repeatedly challenge localization.

Failure handling is equally important. Dense vegetation, heavy rain, dust, reflective surfaces, GNSS outages, sensor vibration, or temporary construction may reduce localization quality. The AMR should recognize degraded confidence and transition to an appropriate operational state, such as reduced speed, localization recovery, safe stopping, alternative-route execution, or remote assistance, instead of continuing normal autonomous motion with an unreliable pose.

Production validation should therefore include repeated campus loops, operation at different times of day, seasonal vegetation changes, GNSS-denied sections, slopes, high-vibration surfaces, dynamic traffic, and routes between buildings. Ground-truth checkpoints or surveyed landmarks can be used to quantify absolute and relative errors, while repeated mission results reveal whether localization remains sufficiently consistent for practical autonomous operation.

In a mature outdoor AMR system, LIO-SAM is not merely a point-cloud generation algorithm but part of a broader localization infrastructure. LiDAR provides geometric structure, the IMU maintains high-frequency motion awareness, GNSS supplies global reference, and factor-graph optimization integrates these constraints across time. Map governance, monitoring, recovery logic, navigation, and fleet coordination transform this estimation capability into a dependable campus-scale autonomy system.

대규모 캠퍼스에서 운용되는 실외 자율이동로봇(AMR, Autonomous Mobile Robot)은 장거리 이동, 불규칙한 지형, 변화하는 조명, 식생, 건물, 부분적으로 구조화된 도로 환경에서도 안정적인 위치추정(Localization)을 유지해야 한다. LIO-SAM은 팩터 그래프(Factor Graph) 프레임워크에서 라이다(LiDAR)와 관성 측정값(Inertial Measurement)을 긴밀하게 결합하여 AMR이 움직임을 추정하면서 기하학적으로 일관된 3차원 지도를 구축할 수 있는 실용적인 기반을 제공한다.

일반적인 캠퍼스 지도작성 플랫폼(Mapping Platform)은 3D 라이다(3D LiDAR), 관성측정장치(IMU, Inertial Measurement Unit), 위성항법시스템(GNSS, Global Navigation Satellite System) 수신기, 휠 오도메트리(Wheel Odometry), 온보드 컴퓨팅 장치(Onboard Computing Unit)를 탑재한다. 라이다는 주변의 기하학적 구조를 획득하고 IMU는 고주파 각속도와 선형 가속도를 측정한다. GNSS는 위성 가시성이 충분한 경우 전역 위치 기준을 제공하며, 휠 오도메트리는 안정적인 지면 접촉 상태에서 차량 움직임에 대한 보완 정보를 제공할 수 있다.

지도작성을 시작하기 전에 정확한 센서 보정(Sensor Calibration)이 필수적이다. 라이다와 IMU 사이의 강체 변환(Rigid-body Transformation)을 정밀하게 결정해야 하며, 모션 보정(Motion Compensation)이 가능하도록 두 센서의 타임스탬프(Timestamp)를 충분한 정확도로 동기화해야 한다. 잘못된 외부 파라미터(Extrinsic Parameter)나 시간 오프셋(Timing Offset)은 왜곡된 포인트 클라우드(Point Cloud), 불일치하는 스캔 정합(Scan Alignment), 체계적인 궤적 오차를 발생시키며, AMR이 대규모 캠퍼스를 장거리 이동할수록 이러한 문제가 더욱 명확하게 나타난다.

LIO-SAM은 IMU 측정값을 이용하여 단기 움직임을 추정하고 각각의 라이다 스캔에서 발생하는 모션 왜곡(Motion Distortion)을 보정한다. 이러한 과정은 진동, 가속, 조향, 불규칙한 지형으로 인해 센서의 자세가 지속적으로 변하는 실외 AMR에서 특히 중요하다. 왜곡 보정된 포인트 클라우드(Deskewed Point Cloud)는 이후 특징 추출(Feature Extraction), 스캔 정합(Scan Registration), 팩터 그래프 최적화(Factor Graph Optimization)에 더욱 신뢰할 수 있는 기하학적 관측값을 제공한다.

라이다 관측값은 로봇 포즈(Pose) 사이의 기하학적 제약조건(Geometric Constraint)을 제공한다. 건물, 연석, 벽, 기둥, 노면과 같은 지속적인 구조물의 모서리 및 평면 특징은 스캔 정합을 지원할 수 있다. 그러나 실외 캠퍼스에는 식생, 주차 차량, 보행자, 배송 차량, 건설 장비 등이 존재할 수 있다. 따라서 지도작성 품질은 일시적인 객체에 대한 의존도를 줄이고 안정적인 기하학적 구조를 얼마나 효과적으로 활용하는지에 따라 달라진다.

팩터 그래프(Factor Graph)는 LIO-SAM의 핵심적인 상태 추정(Estimation) 구조이다. 로봇 상태는 그래프 변수(Graph Variable)로 표현되며, IMU 사전적분(IMU Preintegration), 라이다 오도메트리(LiDAR Odometry), 루프 폐쇄(Loop Closure), 사용 가능한 GNSS 관측값이 각 상태 사이의 제약조건을 형성한다. 최적화 과정은 이러한 정보를 결합하여 단순히 지역적인 움직임 추정값을 연속적으로 적분하는 방법보다 전역적으로 일관된 궤적(Global Consistent Trajectory)을 추정한다.

대규모 캠퍼스에서는 누적 드리프트(Accumulated Drift)의 관리가 특히 중요하다. 정확한 라이다-관성 오도메트리(LiDAR-Inertial Odometry)도 수백 또는 수천 미터를 이동하면 점진적으로 오차가 누적된다. AMR이 이전에 지도화했던 영역을 다시 방문하면 루프 폐쇄 검출(Loop Closure Detection)을 통해 대응 관계를 식별하고 장거리 제약조건(Long-range Constraint)을 추가할 수 있다. 이후 그래프 최적화(Graph Optimization)는 전체 궤적에 누적 오차를 재분배하여 캠퍼스 지도의 서로 다른 영역 사이의 일관성을 향상시킨다.

GNSS는 지역적으로 정확한 SLAM 궤적을 전역 지리 좌표 기준(Global Geographic Reference)에 고정함으로써 루프 폐쇄를 보완할 수 있다. 일반 GNSS는 유용한 대략적 제약조건을 제공하며, RTK-GNSS(Real-Time Kinematic GNSS)는 적절한 위성 및 보정 조건에서 훨씬 높은 위치 정확도를 제공할 수 있다. 그러나 다중경로(Multipath), 건물, 나무, 터널, 도심 협곡(Urban Canyon)으로 GNSS 신뢰성이 일시적으로 저하될 수 있으므로 전역 측정값은 불확실성(Uncertainty)을 고려하여 통합해야 한다.

따라서 실제 생산 시스템에서는 GNSS를 절대적으로 신뢰할 수 있는 지상진실(Ground Truth)로 취급하지 않는 아키텍처가 유용하다. LIO-SAM은 상대적으로 정확한 라이다-관성 추정값과 적절한 가중치를 적용한 전역 관측값을 결합하여 손상된 GNSS 측정값을 무조건 추종하지 않으면서 GNSS의 장점을 활용할 수 있다. GNSS 공분산(Covariance), 고정 상태(Fix Status), 위성 기하구조(Satellite Geometry), 보정정보 가용성(Correction Availability)을 모니터링하면 전역 측정값이 최적화된 궤적에 어느 정도 영향을 주어야 하는지 판단할 수 있다.

캠퍼스 규모의 지도작성에서는 세심한 좌표 관리(Coordinate Management)도 필요하다. SLAM 지도는 일반적으로 지역 직교 좌표계(Local Cartesian Coordinate System)를 사용하는 반면 GNSS 관측값은 지리 좌표 기준(Geographic Reference System)을 사용한다. 지리 좌표와 동-북-상(ENU, East-North-Up)과 같은 지역 좌표계 사이의 명확한 변환을 정의하면 모든 지역 계산을 위도와 경도로 직접 수행하지 않고도 지도, 임무, 인프라 위치, 플릿 목적지가 일관된 공간 기준(Spatial Reference)을 공유할 수 있다.

지형 변화(Terrain Variation)는 실외 AMR에 추가적인 문제를 발생시킨다. 경사로, 과속방지턱, 연석, 배수로, 자갈길, 잔디, 손상된 포장도로에서는 이상적인 평면 내비게이션 모델에서 거의 나타나지 않는 롤(Roll), 피치(Pitch), 수직 운동(Vertical Motion), 휠 슬립(Wheel Slip)이 발생한다. 3차원 라이다-관성 상태 추정은 차량의 6자유도 운동(6-DoF Motion)을 유지하므로 완전히 평평한 운용 표면을 가정하는 위치추정 아키텍처보다 이러한 환경에 더욱 적합하다.

생산용 지도작성 임무(Production Mapping Mission)는 단순히 각 경로를 한 번 주행하는 것이 아니라 중요한 도로와 운용 영역을 여러 관점에서 관측하도록 구성해야 한다. 교차로, 건물 출입구, 충전 구역, 좁은 통로, 주요 순환 경로를 반복적으로 방문하면 중복된 기하학적 제약조건을 확보할 수 있다. 이러한 반복 관측은 루프 폐쇄를 강화하며 환경 특징이 부족하거나 반복적이어서 위치추정 품질이 낮은 지도 영역을 식별할 기회를 제공한다.

지도 밀도(Map Density)는 연산량과 저장 공간 요구사항 사이에서 균형을 유지해야 한다. 대규모 캠퍼스에서 수집된 원시 라이다 데이터(Raw LiDAR Data)는 매우 큰 용량을 차지할 수 있지만, 자율주행에는 모든 측정 포인트가 필요한 것은 아니다. 복셀 필터링(Voxel Filtering), 키프레임 선택(Keyframe Selection), 지역 서브맵(Local Submap), 최적화된 포인트 클라우드 표현(Point-cloud Representation)을 이용하면 위치추정, 장애물 해석, 엔지니어링 검증에 필요한 구조를 보존하면서 데이터 크기를 줄일 수 있다.

생성된 3차원 지도는 명확한 버전(Version)과 검증 상태(Validation Status)를 가진 생산 자산(Production Asset)으로 관리해야 한다. 건물, 도로, 연석, 기둥, 고정 인프라, 도킹 영역(Docking Area), 충전소(Charging Station), 제한 구역(Restricted Region), 임무 랜드마크(Mission Landmark)를 기하학적 지도와 연계할 수 있다. 공사, 조경 변경, 장비 재배치, 신규 건물 등으로 환경이 변화하면 운영 기준 지도를 자동으로 수정하기보다는 통제된 평가 절차를 수행해야 한다.

일상적인 운영에서의 위치추정은 캠퍼스 전체를 지속적으로 다시 구축하는 대신 검증된 지도를 재사용할 수 있다. 현재의 라이다와 IMU 측정값을 통해 지역 움직임을 추정하고 AMR을 기존 환경에 정합시키며, 필요한 경우 GNSS가 전역 위치추정을 보조한다. 이러한 아키텍처는 불필요한 지도 변경을 감소시키고 여러 AMR이 동일한 공간 기준을 공유하면서도 각각 독립적인 상태 추정값(State Estimate)을 유지할 수 있도록 한다.

운영 모니터링(Operational Monitoring)은 단순히 SLAM 소프트웨어가 실행되고 있는지만 확인해서는 안 된다. 라이다 정합 품질(LiDAR Registration Quality), IMU 바이어스(IMU Bias), 추정 공분산(Estimated Covariance), GNSS 잔차(GNSS Residual), 루프 폐쇄 이벤트, 지도 정합 성능(Map Matching Performance), 궤적 불연속(Trajectory Discontinuity), 처리 지연시간(Processing Latency), 위치추정 복구 빈도를 시스템 상태 지표로 활용할 수 있다. 장기 통계는 센서 성능 저하, 보정값 변화, 환경 변화 또는 반복적으로 위치추정에 문제를 일으키는 캠퍼스 영역을 식별하는 데 활용할 수 있다.

장애 처리(Failure Handling) 역시 중요하다. 밀집된 식생, 폭우, 먼지, 반사 표면, GNSS 단절, 센서 진동, 임시 공사 등은 위치추정 품질을 저하시킬 수 있다. AMR은 신뢰도가 저하된 상태를 인식하고 신뢰할 수 없는 포즈를 기반으로 정상 자율주행을 계속하기보다는 감속(Reduced Speed), 위치추정 복구(Localization Recovery), 안전 정지(Safe Stop), 대체 경로 실행(Alternative-route Execution), 원격 지원(Remote Assistance)과 같은 적절한 운영 상태로 전환해야 한다.

따라서 생산 검증(Production Validation)에서는 반복적인 캠퍼스 순환 경로, 서로 다른 시간대의 운용, 계절별 식생 변화, GNSS 음영 구간(GNSS-denied Section), 경사로, 고진동 노면, 동적 교통 환경, 건물 사이의 이동 경로 등을 포함해야 한다. 지상진실 기준점(Ground-truth Checkpoint)이나 측량된 랜드마크(Surveyed Landmark)를 이용하여 절대 및 상대 오차를 정량화할 수 있으며, 반복 임무 결과를 통해 실제 자율주행에 필요한 위치추정 일관성이 유지되는지 평가할 수 있다.

성숙한 실외 AMR 시스템에서 LIO-SAM은 단순한 포인트 클라우드 생성 알고리즘(Point-cloud Generation Algorithm)이 아니라 보다 광범위한 위치추정 인프라(Localization Infrastructure)의 일부이다. 라이다는 기하학적 구조를 제공하고, IMU는 고주파 움직임 상태를 유지하며, GNSS는 전역 기준을 제공하고, 팩터 그래프 최적화는 시간에 따라 이러한 제약조건을 통합한다. 지도 거버넌스(Map Governance), 모니터링(Monitoring), 복구 로직(Recovery Logic), 내비게이션(Navigation), 플릿 조정(Fleet Coordination)이 결합될 때 이러한 상태 추정 역량은 신뢰할 수 있는 캠퍼스 규모의 자율주행 시스템으로 확장될 수 있다.

##  

## 12.03. Mobile Manipulator 3D Workspace Mapping Case

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

A mobile manipulator combines autonomous navigation with robotic manipulation, requiring a spatial representation that supports both meter-scale base motion and centimeter- or millimeter-scale interaction near objects. Three-dimensional workspace mapping therefore extends conventional mobile-robot SLAM by representing shelves, tables, machines, containers, fixtures, and manipulation targets with sufficient geometric consistency for perception, planning, and control.

A typical platform integrates an AMR base, multi-axis manipulator, RGB-D or stereo cameras, 3D LiDAR, joint encoders, IMU, and onboard computing. LiDAR provides robust structural geometry over relatively large distances, while cameras capture dense local shape, color, and semantic information. Joint encoders determine manipulator configuration, allowing observations from wrist-mounted sensors to be transformed into the robot and map coordinate systems.

Coordinate-frame management is fundamental because the system contains multiple moving bodies. The map, odometry, mobile base, manipulator base, individual joints, end effector, cameras, and LiDAR must maintain consistent transformations. Errors in these relationships directly affect object position and grasp execution. A small extrinsic calibration error can become significant when projected through a long manipulator reach or when manipulating compact components.

The mobile base first requires reliable localization within the facility. LiDAR SLAM, visual-inertial SLAM, or multi-sensor localization can estimate the base pose relative to a persistent map. Once the base reaches a target workstation, local perception refines the geometry around the manipulation area. This hierarchical approach avoids requiring the facility-wide SLAM system to provide the precision needed for every grasp or insertion operation.

Three-dimensional workspace reconstruction begins by integrating observations acquired from different viewpoints. Depth cameras or LiDAR generate point clouds that are transformed into a common reference frame using estimated sensor poses. Multiple measurements can then be fused into voxel maps, occupancy representations, surfel maps, point-cloud models, or signed-distance fields depending on the requirements of collision checking, surface reconstruction, and manipulation planning.

Active perception becomes important when a single viewpoint cannot reveal enough of the workspace. Shelves, containers, machinery, and stacked objects frequently produce occlusions. The mobile base may reposition itself, the manipulator may move a wrist camera, or both may cooperate to observe hidden surfaces. The mapping system incrementally incorporates these measurements while maintaining the spatial relationship between previously observed and newly visible structures.

Workspace maps should distinguish static structures from movable objects. Walls, workbenches, fixed machines, racks, and safety barriers can remain part of a persistent geometric map, while boxes, tools, workpieces, trays, and human-operated equipment may change frequently. Treating all observations as permanent would rapidly produce inconsistent maps, so production systems often combine a stable background representation with continuously updated local object states.

Semantic perception adds operational meaning to geometry. A point cloud may describe a rectangular surface, but manipulation requires understanding whether that surface belongs to a table, pallet, storage bin, machine interface, or target component. Object detection, segmentation, pose estimation, and instance tracking can therefore be associated with the geometric map, producing a semantic workspace model that supports task-level reasoning and manipulation.

Object pose accuracy is especially important near the end effector. Global localization may be sufficient to bring the robot within reach of a workstation, but final manipulation should normally rely on local observations. RGB-D cameras, stereo vision, fiducial markers, structured light, or dedicated industrial vision can estimate the target relative to the manipulator. This reduces dependence on accumulated global map error during precise interaction.

The manipulator planning system converts the mapped workspace into collision information. Occupied voxels, reconstructed surfaces, or simplified collision meshes describe shelves, tables, machines, and nearby obstacles. Motion planning then searches for a joint trajectory that moves the end effector toward the target while avoiding collisions with the environment, the mobile base, and potentially the robot\'s own links.

A mobile manipulator introduces an additional planning problem because both the base and arm influence reachability. A target may be visible but unreachable from the current base pose, or an apparently convenient base position may leave insufficient arm clearance. Workspace mapping can therefore support base-placement planning by evaluating candidate robot positions according to reachability, visibility, collision risk, manipulability, and navigation accessibility.

Navigation and manipulation should be coordinated rather than treated as unrelated behaviors. The AMR can first navigate to a coarse approach pose, perform local workspace mapping, determine an improved manipulation pose, and make a small base adjustment before arm motion begins. After completing the task, the manipulator can return to a safe transport configuration before the mobile base resumes facility navigation.

Dynamic environments require continuous local map updates. Workers may enter the workspace, carts may move nearby, or objects may be removed while the robot is planning. Depth and LiDAR sensors should refresh collision information sufficiently quickly for the system to detect meaningful changes. A planned trajectory must be reconsidered when newly observed obstacles invalidate the assumptions used during motion planning.

Map resolution should vary according to task requirements. Large structural areas do not require the same geometric density as a grasping region containing small objects. Multi-resolution representations can preserve coarse facility geometry while maintaining finer information around manipulation targets. This reduces memory and computation without sacrificing the precision required for close-range collision avoidance and object interaction.

Uncertainty should also be represented explicitly. Sensor noise, calibration error, localization drift, reflective surfaces, transparent materials, and incomplete observations can make apparently precise geometry unreliable. Confidence measures associated with object poses, depth observations, or map regions allow the system to determine whether it can execute a manipulation directly or should collect additional observations before committing to motion.

Production operation requires monitoring of both mapping and manipulation quality. Localization confidence, depth completeness, object-pose uncertainty, calibration status, transform latency, collision-map age, planning success rate, grasp success, and repeated correction motions provide useful health indicators. Trends in these measurements can reveal sensor degradation, mechanical changes, or workstations whose geometry has changed since commissioning.

Failure recovery should exploit the mobility of the system. If an object cannot be detected reliably, the robot can change viewpoint rather than immediately terminating the task. If inverse kinematics fails, the base can move to a different position. If the workspace has changed substantially, the robot can rebuild the local map. Only when these recovery strategies fail should the system escalate to operator assistance or task reassignment.

Validation must evaluate complete mobile-manipulation missions rather than isolated SLAM accuracy. Tests should include navigation to multiple workstations, repeated base positioning, local reconstruction, object localization, grasping, placement, occlusion recovery, dynamic obstacle response, and operation after map or workstation changes. Repeatability across many missions is particularly important because small spatial errors can accumulate through the perception-to-action chain.

Fleet operation introduces another layer of spatial coordination. Multiple mobile manipulators may share a common facility map while maintaining individual local workspace models. The fleet manager can assign tasks according to robot availability, tool configuration, battery state, and workstation accessibility, while each robot independently refines the geometry required for its current manipulation task.

A mature 3D workspace mapping architecture therefore connects global SLAM, local reconstruction, semantic perception, object pose estimation, motion planning, and closed-loop manipulation. The global map brings the robot to the correct operational region, while high-resolution local sensing establishes the spatial accuracy required for interaction. Together, these layers transform a mobile platform and robotic arm into a system capable of autonomous physical work in changing industrial environments.

모바일 매니퓰레이터(Mobile Manipulator)는 자율주행과 로봇 조작을 결합하므로 미터(m) 단위의 이동뿐만 아니라 물체 주변에서 센티미터(cm) 또는 밀리미터(mm) 수준의 정밀한 상호작용을 지원하는 공간 표현이 필요하다. 따라서 3차원 작업공간 지도작성(3D Workspace Mapping)은 기존 모바일 로봇 SLAM을 확장하여 선반, 테이블, 기계, 컨테이너, 고정구, 조작 대상물 등을 인식하고, 인식·계획·제어에 충분한 기하학적 일관성을 유지할 수 있도록 해야 한다.

일반적인 플랫폼은 AMR 베이스(AMR Base), 다축 매니퓰레이터(Multi-axis Manipulator), RGB-D 또는 스테레오 카메라(Stereo Camera), 3D 라이다(3D LiDAR), 관절 엔코더(Joint Encoder), IMU, 온보드 컴퓨팅(Onboard Computing)을 통합한다. 라이다는 비교적 넓은 거리에서 강건한 구조 정보를 제공하며, 카메라는 조밀한 형상, 색상, 의미 정보를 획득한다. 관절 엔코더는 매니퓰레이터의 구성을 결정하여 손목 장착 센서(Wrist-mounted Sensor)의 관측값을 로봇 및 지도 좌표계(Map Coordinate System)로 변환할 수 있도록 한다.

좌표 프레임 관리(Coordinate-frame Management)는 여러 개의 움직이는 구성요소를 포함하는 시스템에서 핵심적인 요소이다. 지도(Map), 오도메트리(Odometry), 이동 베이스(Mobile Base), 매니퓰레이터 베이스(Manipulator Base), 각 관절(Joint), 엔드 이펙터(End Effector), 카메라, 라이다 사이의 변환 관계(Transform)를 일관되게 유지해야 한다. 이러한 관계의 오차는 물체 위치와 그립(Grasp) 실행에 직접적인 영향을 준다. 작은 외부 파라미터 보정 오차(Extrinsic Calibration Error)도 긴 매니퓰레이터의 도달 거리나 소형 부품을 조작하는 과정에서는 상당한 위치 오차로 확대될 수 있다.

이동 베이스는 먼저 시설 내부에서 신뢰할 수 있는 위치추정(Localization)을 확보해야 한다. 라이다 SLAM(LiDAR SLAM), 시각-관성 SLAM(Visual-Inertial SLAM), 또는 다중 센서 위치추정(Multi-sensor Localization)을 이용하여 지속적으로 유지되는 지도에 대한 베이스의 포즈(Pose)를 추정할 수 있다. AMR이 작업 스테이션(Workstation)에 도착하면 근거리 인식(Local Perception)을 통해 조작 영역의 기하학적 정보를 더욱 정밀하게 보정한다. 이러한 계층적 접근법은 시설 전체 SLAM 시스템이 모든 그립 동작에 필요한 정밀도를 직접 제공해야 하는 부담을 줄여준다.

3차원 작업공간 재구성(3D Workspace Reconstruction)은 서로 다른 관점에서 획득한 관측값을 통합하는 것에서 시작한다. 깊이 카메라(Depth Camera) 또는 라이다는 포인트 클라우드(Point Cloud)를 생성하며, 추정된 센서 포즈를 이용하여 이를 공통 기준 좌표계(Common Reference Frame)로 변환한다. 이후 여러 측정값을 복셀 지도(Voxel Map), 점유 표현(Occupancy Representation), 서펠 지도(Surfel Map), 포인트 클라우드 모델(Point-cloud Model), 또는 부호 거리장(Signed Distance Field) 등으로 융합할 수 있으며, 선택된 표현은 충돌 검사(Collision Checking), 표면 재구성(Surface Reconstruction), 조작 계획(Manipulation Planning)의 요구사항에 따라 결정된다.

단일 시점에서 작업공간을 충분히 관측할 수 없는 경우 능동 인식(Active Perception)이 중요해진다. 선반, 컨테이너, 기계, 적재된 물체는 빈번하게 가림(Occlusion)을 발생시킨다. 이동 베이스가 위치를 변경하거나 매니퓰레이터가 손목 카메라(Wrist Camera)를 이동시키거나 두 가지를 함께 수행하여 가려진 표면을 관측할 수 있다. 지도작성 시스템은 이러한 측정값을 점진적으로 통합하면서 기존에 관측된 구조와 새롭게 관측된 구조 사이의 공간적 관계를 유지해야 한다.

작업공간 지도(Workspace Map)는 고정 구조물과 이동 가능한 객체를 구분해야 한다. 벽, 작업대, 고정 기계, 랙, 안전 펜스 등은 지속적인 기하학적 지도(Persistent Geometric Map)의 일부로 유지할 수 있지만, 상자, 공구, 작업물, 트레이, 작업자가 사용하는 장비 등은 자주 변경될 수 있다. 모든 관측값을 영구적인 정보로 처리하면 지도는 빠르게 불일치하게 되므로, 생산 시스템에서는 일반적으로 안정적인 배경 표현(Stable Background Representation)과 지속적으로 갱신되는 지역 객체 상태(Local Object State)를 결합한다.

의미론적 인식(Semantic Perception)은 기하학 정보에 실제 작업상의 의미를 부여한다. 포인트 클라우드는 단순히 직사각형 형태의 표면을 나타낼 수 있지만, 조작 시스템은 그 표면이 테이블인지, 팔레트인지, 저장 컨테이너인지, 기계 인터페이스인지, 또는 목표 부품인지 이해해야 한다. 따라서 객체 검출(Object Detection), 분할(Segmentation), 포즈 추정(Pose Estimation), 인스턴스 추적(Instance Tracking)을 기하학적 지도와 연결하여 작업 수준 추론(Task-level Reasoning)과 조작을 지원하는 의미론적 작업공간 모델(Semantic Workspace Model)을 구축할 수 있다.

엔드 이펙터 주변에서는 객체 포즈 정확도(Object Pose Accuracy)가 특히 중요하다. 전역 위치추정(Global Localization)은 로봇을 작업 스테이션의 도달 가능한 영역까지 이동시키는 데 충분할 수 있지만, 최종 조작은 일반적으로 지역 관측(Local Observation)에 의존해야 한다. RGB-D 카메라, 스테레오 비전(Stereo Vision), 기준 마커(Fiducial Marker), 구조광(Structured Light), 전용 산업용 비전(Industrial Vision)을 이용하여 매니퓰레이터에 대한 목표물의 상대 위치를 추정할 수 있다. 이를 통해 정밀한 상호작용 과정에서 누적된 전역 지도 오차에 대한 의존도를 줄일 수 있다.

매니퓰레이터 계획 시스템(Manipulator Planning System)은 지도화된 작업공간을 충돌 정보(Collision Information)로 변환한다. 점유 복셀(Occupied Voxel), 재구성된 표면(Reconstructed Surface), 단순화된 충돌 메시(Collision Mesh)는 선반, 테이블, 기계, 주변 장애물을 표현한다. 이후 모션 계획(Motion Planning)은 주변 환경, 이동 베이스, 그리고 로봇 자체의 각 링크(Link)와 충돌하지 않으면서 엔드 이펙터를 목표 위치로 이동시키는 관절 궤적(Joint Trajectory)을 탐색한다.

모바일 매니퓰레이터에서는 베이스와 매니퓰레이터가 모두 도달 가능성에 영향을 주기 때문에 추가적인 계획 문제가 발생한다. 목표물이 보이는 상태라도 현재 베이스 포즈에서는 도달할 수 없을 수 있으며, 반대로 편리해 보이는 베이스 위치가 매니퓰레이터의 작업 공간이나 링크 간 여유 공간을 충분히 확보하지 못할 수도 있다. 따라서 작업공간 지도는 도달 가능성(Reachability), 가시성(Visibility), 충돌 위험(Collision Risk), 조작성(Manipulability), 내비게이션 접근성(Navigation Accessibility)을 기준으로 후보 로봇 위치를 평가하여 베이스 위치 계획(Base-placement Planning)을 지원할 수 있다.

내비게이션(Navigation)과 조작(Manipulation)은 서로 관련 없는 동작으로 분리하기보다 상호 연계하여 수행해야 한다. AMR은 먼저 대략적인 접근 포즈(Coarse Approach Pose)까지 이동한 후 지역 작업공간 지도를 작성하고, 개선된 조작 위치를 결정하며, 팔의 동작이 시작되기 전에 작은 베이스 위치 조정을 수행할 수 있다. 작업이 완료되면 매니퓰레이터는 안전한 운송 자세(Safe Transport Configuration)로 복귀한 후 이동 베이스가 다시 시설 내 주행을 시작하도록 할 수 있다.

동적 환경에서는 지역 지도의 지속적인 갱신(Continuous Local Map Update)이 필요하다. 작업자가 작업공간에 들어오거나, 카트가 주변을 이동하거나, 로봇이 계획하는 동안 물체가 제거될 수 있다. 깊이 센서와 라이다 센서는 충돌 정보를 충분히 빠르게 갱신하여 의미 있는 환경 변화를 검출해야 한다. 새롭게 관측된 장애물이 기존의 이동 계획을 무효화한다면 계획된 궤적을 다시 계산해야 한다.

지도 해상도(Map Resolution)는 작업 요구사항에 따라 달라져야 한다. 대규모 구조 영역은 작은 물체가 존재하는 조작 영역과 동일한 기하학적 밀도를 필요로 하지 않는다. 다중 해상도 표현(Multi-resolution Representation)을 사용하면 시설 전체의 거친 기하학 정보를 유지하면서 조작 대상 주변에는 더욱 세밀한 정보를 유지할 수 있다. 이를 통해 근거리 충돌 회피와 객체 상호작용에 필요한 정밀도를 유지하면서 메모리와 연산량을 줄일 수 있다.

불확실성(Uncertainty) 역시 명시적으로 표현하는 것이 바람직하다. 센서 노이즈, 보정 오차, 위치추정 드리프트, 반사 표면, 투명 재질, 불완전한 관측은 겉보기에는 정밀해 보이는 기하학 정보도 신뢰하기 어렵게 만들 수 있다. 객체 포즈, 깊이 관측값, 지도 영역에 대한 신뢰도(Confidence)를 함께 관리하면 시스템이 즉시 조작을 수행할 수 있는지 또는 동작을 실행하기 전에 추가 관측을 수행해야 하는지를 판단할 수 있다.

생산 환경에서는 지도작성 품질과 조작 품질을 모두 모니터링해야 한다. 위치추정 신뢰도(Localization Confidence), 깊이 정보 완전성(Depth Completeness), 객체 포즈 불확실성(Object Pose Uncertainty), 보정 상태(Calibration Status), 좌표 변환 지연시간(Transform Latency), 충돌 지도의 최신성(Collision-map Age), 계획 성공률(Planning Success Rate), 그립 성공률(Grasp Success Rate), 반복적인 보정 동작(Correction Motion) 등을 유용한 상태 지표로 활용할 수 있다. 이러한 지표의 추세를 분석하면 센서 성능 저하, 기계적 변화 또는 초기 구축 이후 형상이 변경된 작업 스테이션을 식별할 수 있다.

장애 복구(Failure Recovery)는 시스템의 이동성을 적극적으로 활용해야 한다. 객체를 안정적으로 검출할 수 없다면 즉시 작업을 종료하기보다 로봇의 관점을 변경할 수 있다. 역기구학(IK, Inverse Kinematics)이 실패하면 베이스를 다른 위치로 이동할 수 있다. 작업공간이 크게 변경되었다면 로봇이 지역 지도를 다시 구축할 수 있다. 이러한 복구 전략으로도 문제가 해결되지 않는 경우에만 작업자 지원(Operator Assistance)이나 작업 재할당(Task Reassignment)으로 확대하는 것이 적절하다.

검증(Validation)은 개별적인 SLAM 정확도만 평가하기보다 전체 모바일 조작 임무(Mobile Manipulation Mission)를 대상으로 수행해야 한다. 여러 작업 스테이션으로의 내비게이션, 반복적인 베이스 위치 결정, 지역 공간 재구성, 객체 위치추정, 그립, 배치, 가림 복구(Occlusion Recovery), 동적 장애물 대응, 지도 또는 작업 스테이션 변경 이후의 운용 등을 포함해야 한다. 특히 인식에서 행동으로 이어지는 과정에서 작은 공간 오차가 누적될 수 있으므로 많은 반복 임무에서의 재현성(Repeatability)을 평가하는 것이 중요하다.

플릿 운용(Fleet Operation)에서는 추가적인 공간 조정 계층(Spatial Coordination Layer)이 필요하다. 여러 모바일 매니퓰레이터가 공통 시설 지도를 공유하면서 각각 독립적인 지역 작업공간 모델을 유지할 수 있다. 플릿 관리자(Fleet Manager)는 로봇 가용성, 도구 구성, 배터리 상태, 작업 스테이션 접근성을 고려하여 임무를 할당하며, 각 로봇은 현재 수행 중인 조작 작업에 필요한 기하학 정보를 독립적으로 정밀화할 수 있다.

성숙한 3차원 작업공간 지도작성 아키텍처(3D Workspace Mapping Architecture)는 전역 SLAM, 지역 재구성(Local Reconstruction), 의미론적 인식, 객체 포즈 추정, 모션 계획, 폐루프 조작(Closed-loop Manipulation)을 하나의 연속적인 구조로 연결한다. 전역 지도는 로봇을 올바른 작업 영역으로 이동시키고, 고해상도 지역 센싱은 실제 상호작용에 필요한 공간 정밀도를 확보한다. 이러한 계층이 결합되면 모바일 플랫폼과 로봇 팔은 변화하는 산업 환경에서 실제 물리적 작업을 자율적으로 수행할 수 있는 시스템으로 발전할 수 있다.

##  

## 12.04. Quadruped Legged SLAM Rough Terrain Case

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

Quadruped robots extend autonomous mobility into environments where wheels cannot reliably maintain contact, including rubble, stairs, rocks, steep slopes, industrial structures, construction sites, and natural terrain. SLAM for these platforms must estimate six-degree-of-freedom motion while tolerating rapid body oscillation, repeated impacts, changing footholds, and substantial roll, pitch, and vertical displacement.

A typical legged SLAM platform combines a 3D LiDAR, IMU, stereo or RGB-D cameras, joint encoders, and foot-contact information. LiDAR provides stable geometric observations over useful distances, cameras add texture and semantic information, and the IMU measures high-frequency body motion. Joint positions and contact states provide additional constraints that are unavailable to conventional wheeled robots.

Sensor placement is especially important because quadruped locomotion continuously changes the body attitude. A LiDAR mounted near the upper body can maintain broad visibility, while cameras may cover forward terrain and close-range footholds. Accurate extrinsic calibration between the body, LiDAR, cameras, and IMU is essential because small rotational errors can generate substantial spatial errors when observations are projected over distance.

Time synchronization is equally critical. Walking, trotting, climbing, and recovering from disturbances generate rapid rotational and translational motion. Even a modest timestamp mismatch between LiDAR and IMU measurements can distort point clouds and degrade scan registration. Hardware synchronization or carefully characterized software synchronization should therefore be considered part of the localization architecture rather than an optional refinement.

LiDAR-inertial odometry is well suited to rough-terrain locomotion because the IMU captures rapid motion between successive LiDAR observations. IMU integration predicts short-term orientation and displacement, while LiDAR registration corrects accumulated inertial drift using surrounding geometry. Motion compensation can deskew each scan so that points measured during body movement are represented within a consistent temporal reference.

Leg kinematics provide another useful source of motion information. When a foot is firmly contacting the ground without slipping, joint encoder measurements and the robot\'s kinematic model can constrain body movement relative to the contact point. Combining these constraints with LiDAR and IMU estimation can improve robustness, particularly when geometric features are temporarily weak or rapid body motion challenges scan matching.

Foot-contact constraints are not universally reliable. Loose gravel, mud, wet surfaces, vegetation, rubble, and unstable debris can cause slipping or moving footholds. A production estimator should therefore treat leg odometry according to contact confidence rather than assuming every stance foot is fixed. Force sensing, motor torque, kinematic consistency, and contact-state estimation can help determine whether a foothold should contribute strongly to localization.

Terrain geometry must support both localization and locomotion. The same 3D point cloud used for SLAM can be transformed into elevation maps, traversability maps, surface-normal estimates, slope measurements, and obstacle representations. These derived layers allow the robot to distinguish flat ground from steps, rocks, gaps, steep surfaces, and potentially unstable regions before selecting a body path and individual footholds.

Unlike a wheeled AMR, a quadruped cannot represent navigation only as a planar path through free space. The planner must consider whether the robot can physically establish stable contacts along the route. Terrain height, slope, roughness, step dimensions, clearance, friction assumptions, and body orientation can influence traversability. Localization and terrain mapping therefore become tightly coupled with locomotion planning.

Local terrain maps generally require higher resolution than the global SLAM map. A global point-cloud or voxel representation may provide sufficient information for long-range localization and route planning, while the area immediately surrounding the robot requires detailed geometry for foothold selection. Maintaining global and local representations at different resolutions reduces computational cost while preserving information needed for safe locomotion.

Loop closure remains important during long inspection or exploration missions. A quadruped returning to a previously visited corridor, building section, tunnel, or outdoor region can recognize geometric similarity and introduce a loop constraint. Pose-graph optimization then reduces accumulated drift and improves consistency across the mission map, particularly when GNSS is unavailable or unreliable.

GNSS and RTK-GNSS can provide global references during outdoor operation, but rough-terrain missions often include forests, structures, tunnels, industrial plants, and areas close to buildings. Satellite measurements may therefore become intermittent or degraded. A robust system should transition between globally aided and locally autonomous localization without introducing large pose discontinuities when GNSS quality changes.

Visual information can complement LiDAR in environments where geometry alone is ambiguous. Cameras may identify texture, structural features, doors, signs, equipment, or semantic landmarks. Visual-inertial and LiDAR-inertial estimates can be combined within a multi-sensor architecture, providing redundancy when dust, darkness, repetitive geometry, reflective materials, or limited sensor range reduces the reliability of one sensing modality.

Dynamic body motion introduces additional perception challenges. During a trot or rapid recovery maneuver, sensors can experience vibration, angular acceleration, and temporary occlusion by robot limbs. Mechanical isolation, rigid sensor mounting, high-rate inertial sensing, motion-aware filtering, and self-occlusion handling help maintain useful measurements. The mapping pipeline should also avoid treating moving legs as persistent environmental geometry.

Localization uncertainty should directly influence locomotion behavior. When pose confidence decreases, the robot may reduce speed, select more conservative footholds, increase sensing time, or stop at a stable stance. Continuing aggressive locomotion with uncertain terrain alignment can turn a localization problem into a physical stability problem, particularly near stairs, cliffs, gaps, machinery, or narrow elevated structures.

Failure recovery can exploit the quadruped\'s ability to change posture and viewpoint. If scan matching becomes unreliable, the robot can stop, adopt a stable stance, rotate its body or sensor field of view, and collect additional observations for relocalization. If a route becomes physically unsafe, it can retreat to a previously validated foothold region or request an alternative path rather than forcing forward motion.

Production monitoring should include localization quality, IMU bias, LiDAR registration residuals, visual tracking status, contact confidence, estimated foot slip, terrain-map freshness, loop closures, and pose uncertainty. Locomotion-related indicators such as unexpected body motion, repeated foothold correction, and abnormal joint loads can also reveal localization or terrain-model problems that are not obvious from SLAM metrics alone.

Validation must reproduce the physical conditions expected during deployment. Tests should include slopes, stairs, gravel, grass, wet surfaces, uneven rocks, narrow passages, low-feature areas, GNSS-denied sections, vibration, dynamic obstacles, and repeated transitions between terrain types. Ground-truth references can quantify trajectory error, while repeated missions reveal whether localization and locomotion remain consistently coupled.

For industrial inspection, the map can also become a reference for mission semantics. Equipment locations, inspection points, restricted zones, stairways, hazardous areas, communication dead zones, and recovery locations can be associated with the geometric representation. The robot can then reason not only about where it is, but also about which inspection task or locomotion behavior is appropriate at a particular location.

Multiple quadrupeds can share a validated global map while maintaining independent local terrain maps and state estimates. Fleet coordination can distribute inspection areas, avoid route conflicts, monitor localization health, and reassign missions when a robot encounters inaccessible terrain. Map updates discovered by one robot can be reviewed before being propagated to the remaining fleet rather than automatically altering the common reference.

A mature rough-terrain SLAM architecture therefore integrates LiDAR, inertial sensing, vision, joint kinematics, contact estimation, terrain reconstruction, and uncertainty-aware planning. SLAM establishes spatial consistency, but dependable legged autonomy emerges only when localization confidence is connected to foothold selection, body motion, recovery behavior, and mission management throughout the complete perception-to-action loop.

4족 보행 로봇(Quadruped Robot)은 바퀴가 안정적으로 접촉을 유지하기 어려운 잔해, 계단, 암석, 급경사, 산업 시설, 건설 현장, 자연 지형 등으로 자율 이동 영역을 확장한다. 이러한 플랫폼의 SLAM은 빠른 몸체 진동, 반복적인 충격, 지속적으로 변화하는 발판 접촉(Foothold), 큰 롤(Roll)·피치(Pitch)·수직 변위를 견디면서 6자유도 운동(6-DoF Motion)을 추정할 수 있어야 한다.

일반적인 보행형 SLAM(Legged SLAM) 플랫폼은 3D 라이다(3D LiDAR), 관성측정장치(IMU), 스테레오 또는 RGB-D 카메라, 관절 엔코더(Joint Encoder), 발 접촉 정보(Foot-contact Information)를 결합한다. 라이다는 유효한 거리에서 안정적인 기하학적 관측값을 제공하고, 카메라는 텍스처(Texture)와 의미론적 정보를 추가하며, IMU는 고주파 몸체 움직임을 측정한다. 관절 위치와 접촉 상태는 일반적인 바퀴형 로봇에서는 얻기 어려운 추가적인 제약조건을 제공한다.

센서 배치(Sensor Placement)는 4족 보행 과정에서 몸체 자세가 지속적으로 변화하기 때문에 특히 중요하다. 몸체 상단에 장착된 라이다는 넓은 시야를 유지할 수 있으며, 카메라는 전방 지형과 근거리 발판을 관측하도록 배치할 수 있다. 몸체, 라이다, 카메라, IMU 사이의 정확한 외부 파라미터 보정(Extrinsic Calibration)은 필수적이며, 작은 회전 오차도 장거리 관측값으로 투영되면 상당한 공간 오차를 발생시킬 수 있다.

시간 동기화(Time Synchronization) 역시 매우 중요하다. 걷기(Walking), 트로팅(Trotting), 등반(Climbing), 외란 이후 자세 복구 과정에서는 빠른 회전 및 병진 운동이 발생한다. 라이다와 IMU 측정값 사이에 작은 타임스탬프(Timestamp) 불일치만 존재해도 포인트 클라우드(Point Cloud)가 왜곡되고 스캔 정합(Scan Registration) 성능이 저하될 수 있다. 따라서 하드웨어 동기화 또는 정확하게 특성이 파악된 소프트웨어 동기화는 선택적인 개선 기능이 아니라 위치추정 아키텍처(Localization Architecture)의 일부로 고려해야 한다.

라이다-관성 오도메트리(LiDAR-Inertial Odometry)는 IMU가 연속적인 라이다 관측 사이의 빠른 움직임을 측정할 수 있기 때문에 험지 주행(Rough-terrain Locomotion)에 적합하다. IMU 적분은 단기적인 자세와 변위를 예측하고, 라이다 정합은 주변 기하학 구조를 이용하여 누적되는 관성 드리프트(Inertial Drift)를 보정한다. 모션 보정(Motion Compensation)을 통해 각각의 스캔을 디스큐(Deskew)하면 몸체가 움직이는 동안 측정된 포인트들을 일관된 시간 기준으로 표현할 수 있다.

다리 운동학(Leg Kinematics)은 또 다른 유용한 이동 정보원이 된다. 발이 미끄러지지 않고 지면과 안정적으로 접촉하는 동안에는 관절 엔코더 측정값과 로봇의 운동학 모델(Kinematic Model)을 이용하여 접촉점을 기준으로 몸체 움직임을 제한할 수 있다. 이러한 제약조건을 라이다 및 IMU 기반 상태 추정과 결합하면 기하학적 특징이 일시적으로 부족하거나 빠른 몸체 움직임으로 스캔 정합이 어려워지는 상황에서 강건성(Robustness)을 향상시킬 수 있다.

그러나 발 접촉 제약조건(Foot-contact Constraint)을 항상 신뢰할 수 있는 것은 아니다. 느슨한 자갈, 진흙, 젖은 표면, 식생, 잔해, 불안정한 파편 등에서는 발이 미끄러지거나 발판 자체가 움직일 수 있다. 따라서 생산용 상태 추정기(Estimator)는 모든 지지 발(Stance Foot)이 고정되어 있다고 가정하기보다 접촉 신뢰도(Contact Confidence)에 따라 다리 오도메트리(Leg Odometry)를 반영해야 한다. 힘 센싱(Force Sensing), 모터 토크, 운동학적 일관성, 접촉 상태 추정을 활용하여 각 발판이 위치추정에 얼마나 강하게 기여해야 하는지 판단할 수 있다.

지형 기하학(Terrain Geometry)은 위치추정뿐만 아니라 이동 계획도 지원해야 한다. SLAM에 사용하는 동일한 3D 포인트 클라우드를 고도 지도(Elevation Map), 주행 가능성 지도(Traversability Map), 표면 법선 추정(Surface-normal Estimation), 경사도 측정(Slope Measurement), 장애물 표현(Obstacle Representation)으로 변환할 수 있다. 이러한 파생 계층을 이용하면 로봇이 몸체 경로와 개별 발판을 선택하기 전에 평지, 계단, 암석, 틈, 급경사, 잠재적으로 불안정한 영역을 구분할 수 있다.

바퀴형 AMR과 달리 4족 보행 로봇의 내비게이션(Navigation)은 단순히 자유 공간을 통과하는 평면 경로로만 표현할 수 없다. 계획기는 로봇이 이동 경로를 따라 실제로 안정적인 접촉을 형성할 수 있는지를 고려해야 한다. 지형 높이, 경사, 거칠기, 계단 크기, 여유 공간(Clearance), 마찰 조건, 몸체 자세 등이 주행 가능성(Traversability)에 영향을 준다. 따라서 위치추정과 지형 지도작성(Terrain Mapping)은 보행 계획(Locomotion Planning)과 긴밀하게 결합되어야 한다.

지역 지형 지도(Local Terrain Map)는 일반적으로 전역 SLAM 지도(Global SLAM Map)보다 높은 해상도를 필요로 한다. 전역 포인트 클라우드 또는 복셀 표현(Voxel Representation)은 장거리 위치추정과 경로 계획에 충분한 정보를 제공할 수 있지만, 로봇 주변 영역에서는 발판 선택을 위한 세밀한 기하학 정보가 필요하다. 서로 다른 해상도의 전역 및 지역 표현을 유지하면 안전한 보행에 필요한 정보를 보존하면서 연산 비용을 줄일 수 있다.

루프 폐쇄(Loop Closure)는 장시간의 점검 또는 탐사 임무에서도 중요하다. 4족 보행 로봇이 이전에 방문했던 복도, 건물 구역, 터널 또는 실외 영역으로 돌아오면 기하학적 유사성을 인식하고 루프 제약조건(Loop Constraint)을 추가할 수 있다. 이후 포즈 그래프 최적화(Pose-graph Optimization)는 누적 드리프트를 감소시키고 전체 임무 지도의 일관성을 향상시키며, 특히 GNSS를 사용할 수 없거나 신뢰성이 낮은 환경에서 중요한 역할을 한다.

GNSS와 RTK-GNSS는 실외 운용에서 전역 위치 기준(Global Position Reference)을 제공할 수 있지만, 험지 임무는 숲, 구조물, 터널, 산업 시설, 건물 인접 지역 등을 포함하는 경우가 많다. 따라서 위성 측정값이 간헐적으로 단절되거나 품질이 저하될 수 있다. 강건한 시스템은 GNSS 품질이 변화할 때 큰 포즈 불연속(Pose Discontinuity)을 발생시키지 않으면서 전역 보조 위치추정(Global-aided Localization)과 지역 자율 위치추정(Local Autonomous Localization) 사이를 안정적으로 전환할 수 있어야 한다.

시각 정보(Visual Information)는 기하학 정보만으로 환경을 구분하기 어려운 경우 라이다를 보완할 수 있다. 카메라는 텍스처, 구조적 특징, 문, 표지판, 장비 또는 의미론적 랜드마크(Semantic Landmark)를 식별할 수 있다. 시각-관성 추정(Visual-Inertial Estimation)과 라이다-관성 추정(LiDAR-Inertial Estimation)을 다중 센서 아키텍처(Multi-sensor Architecture)에서 결합하면 먼지, 어둠, 반복적인 기하학 구조, 반사 재질, 제한된 센서 거리 등으로 특정 센서의 신뢰성이 감소하는 상황에서 중복성(Redundancy)을 확보할 수 있다.

동적인 몸체 움직임(Dynamic Body Motion)은 추가적인 인식 문제를 발생시킨다. 트로팅이나 빠른 자세 복구 과정에서는 센서가 진동, 각가속도, 로봇 다리에 의한 일시적인 가림(Self-occlusion)의 영향을 받을 수 있다. 기계적 진동 절연(Mechanical Isolation), 견고한 센서 장착, 고주파 관성 센싱, 움직임 인식 필터링(Motion-aware Filtering), 자체 가림 처리를 통해 유효한 측정값을 유지할 수 있다. 지도작성 파이프라인은 움직이는 로봇 다리를 영구적인 환경 구조로 잘못 처리하지 않아야 한다.

위치추정 불확실성(Localization Uncertainty)은 보행 동작에 직접 반영되어야 한다. 포즈 신뢰도가 감소하면 로봇은 속도를 낮추거나, 더욱 보수적인 발판을 선택하거나, 센싱 시간을 늘리거나, 안정적인 자세에서 정지할 수 있다. 불확실한 지형 정합 상태에서 공격적인 보행을 계속하면 단순한 위치추정 문제가 물리적 안정성 문제로 확대될 수 있으며, 특히 계단, 절벽, 틈, 기계 주변, 좁고 높은 구조물에서는 이러한 위험이 더욱 커진다.

장애 복구(Failure Recovery)는 4족 보행 로봇이 자세와 관측 시점을 변경할 수 있다는 특성을 활용할 수 있다. 스캔 정합의 신뢰성이 떨어지면 로봇은 정지하여 안정적인 자세를 취하고 몸체 또는 센서 시야를 회전시켜 재위치추정(Relocalization)에 필요한 추가 관측값을 수집할 수 있다. 경로가 물리적으로 위험해진 경우에는 무리하게 전진하기보다 이전에 검증된 발판 영역으로 후퇴하거나 대체 경로(Alternative Path)를 요청할 수 있다.

생산 환경 모니터링(Production Monitoring)에서는 위치추정 품질, IMU 바이어스(IMU Bias), 라이다 정합 잔차(LiDAR Registration Residual), 시각 추적 상태(Visual Tracking Status), 접촉 신뢰도, 추정 발 미끄러짐(Estimated Foot Slip), 지형 지도의 최신성(Terrain-map Freshness), 루프 폐쇄, 포즈 불확실성 등을 관리해야 한다. 예상하지 못한 몸체 움직임, 반복적인 발판 보정, 비정상적인 관절 하중과 같은 보행 관련 지표도 SLAM 지표만으로 확인하기 어려운 위치추정 또는 지형 모델 문제를 식별하는 데 활용할 수 있다.

검증(Validation)은 실제 배치 환경에서 예상되는 물리적 조건을 재현해야 한다. 경사로, 계단, 자갈, 잔디, 젖은 표면, 불규칙한 암석, 좁은 통로, 특징이 부족한 영역, GNSS 음영 구간(GNSS-denied Section), 진동, 동적 장애물, 서로 다른 지형 사이의 반복적인 전환 등을 시험해야 한다. 지상진실 기준(Ground-truth Reference)을 이용하여 궤적 오차를 정량화할 수 있으며, 반복 임무를 통해 위치추정과 보행이 지속적으로 안정적으로 결합되는지 평가할 수 있다.

산업 점검(Industrial Inspection)에서는 지도 자체를 임무 의미론(Mission Semantics)을 위한 기준으로 사용할 수도 있다. 장비 위치, 점검 지점(Inspection Point), 제한 구역, 계단, 위험 영역, 통신 음영 구역(Communication Dead Zone), 복구 위치(Recovery Location)를 기하학적 표현과 연결할 수 있다. 이를 통해 로봇은 자신의 위치뿐만 아니라 특정 위치에서 어떤 점검 작업 또는 보행 행동이 적절한지도 판단할 수 있다.

여러 대의 4족 보행 로봇은 검증된 전역 지도(Global Map)를 공유하면서 각각 독립적인 지역 지형 지도와 상태 추정값을 유지할 수 있다. 플릿 조정(Fleet Coordination)은 점검 영역을 분배하고, 경로 충돌을 방지하며, 위치추정 상태를 모니터링하고, 특정 로봇이 접근할 수 없는 지형을 만났을 때 임무를 재할당할 수 있다. 한 로봇이 발견한 지도 변경 사항은 공통 기준 지도를 자동으로 수정하기보다 검토 과정을 거친 후 다른 로봇에 배포하는 것이 바람직하다.

성숙한 험지 SLAM 아키텍처(Rough-terrain SLAM Architecture)는 라이다, 관성 센싱(Inertial Sensing), 비전(Vision), 관절 운동학(Joint Kinematics), 접촉 상태 추정(Contact Estimation), 지형 재구성(Terrain Reconstruction), 불확실성 기반 계획(Uncertainty-aware Planning)을 통합한다. SLAM은 공간적 일관성을 제공하지만, 신뢰할 수 있는 보행 자율성(Legged Autonomy)은 위치추정 신뢰도를 발판 선택, 몸체 움직임, 복구 행동, 임무 관리와 연결하여 전체 인식-행동 루프(Perception-to-action Loop)를 구성할 때 비로소 구현될 수 있다.

##  

## 12.05. Humanoid Visual Inertial SLAM Case

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

Humanoid robots require localization while walking, turning, climbing stairs, manipulating objects, and interacting with environments designed for people. Visual-inertial SLAM is particularly suitable because cameras provide rich geometric and semantic observations while an IMU captures rapid body motion. Together they can estimate six-degree-of-freedom pose without depending continuously on external infrastructure.

A typical humanoid perception system includes stereo or RGB-D cameras, wide-angle navigation cameras, an IMU, joint encoders, and foot-contact sensing. Cameras may be mounted in the head, torso, or wrists according to their function. Head cameras provide long-range environmental perception, while wrist cameras support manipulation. The IMU supplies high-rate angular velocity and acceleration during dynamic body motion.

Humanoid kinematics create localization challenges that differ from wheeled robots. The sensor platform continuously translates and rotates as the pelvis moves, the torso compensates for balance, and the head tracks targets. Walking introduces periodic vertical displacement and oscillation. Visual-inertial estimation must separate this legitimate body motion from sensor noise while preserving a stable estimate of the robot\'s position and orientation.

Accurate calibration between cameras and the IMU is fundamental. Extrinsic calibration determines their relative position and orientation, while temporal calibration aligns image exposure with inertial measurements. Even small timing errors can become significant during rapid head turns or walking. Camera intrinsic parameters, lens distortion, stereo geometry, and rolling-shutter characteristics must also be understood for reliable visual tracking.

Visual-inertial odometry combines complementary sensing properties. The IMU predicts short-term motion at high frequency but accumulates bias and drift, while visual observations provide geometric constraints by tracking features across images. Optimization jointly estimates robot motion, landmarks, velocity, and inertial biases, allowing visual measurements to correct inertial drift and inertial prediction to stabilize tracking during rapid movement.

Feature-based visual SLAM can detect corners, edges, and other distinctive image structures and associate them across frames. In environments with sufficient texture, these correspondences constrain camera motion and support map construction. Direct or semi-direct methods can additionally use image intensity information, potentially providing useful estimates where explicit feature extraction alone is insufficient.

Depth sensing improves geometric understanding around the humanoid. Stereo vision can estimate depth from image disparity, while RGB-D cameras provide direct depth measurements within their operating range. These observations can generate point clouds, voxel maps, occupancy models, or surface representations that support navigation, collision avoidance, stair detection, manipulation, and human-scale interaction with the environment.

Loop closure is necessary for long-duration operation. When the humanoid returns to a previously observed room, corridor, workstation, or building section, visual place recognition can identify the location and create a loop constraint. Pose-graph or bundle-adjustment optimization then reduces accumulated trajectory drift and improves consistency between spatial regions observed at different times.

Leg kinematics can provide additional motion constraints. During a stable stance phase, a supporting foot can temporarily serve as a reference relative to the ground. Joint encoder measurements and the robot\'s kinematic model can estimate body motion with respect to that contact. Combining this information with visual-inertial estimation increases redundancy and can improve localization when visual tracking becomes temporarily weak.

Foot-contact information must nevertheless be treated probabilistically. A humanoid may walk on slippery floors, carpets, ramps, stairs, debris, or compliant surfaces where the support foot does not remain perfectly fixed. Force-torque sensors, pressure sensors, joint torque estimates, and contact-state estimation can help determine whether a foot constraint is reliable enough to influence the localization solution strongly.

Head motion creates both an opportunity and a challenge. Unlike a fixed camera on an AMR, a humanoid can intentionally rotate its head to observe useful landmarks, inspect an uncertain region, or recover visual tracking. Active perception can select viewing directions that increase feature visibility or reduce uncertainty. However, the localization system must correctly account for the kinematic transformation between the moving head and the robot body.

Manipulation introduces additional coordinate relationships. The robot may localize globally using head cameras while wrist cameras estimate the pose of an object relative to the hand. A consistent transform hierarchy connects the global map, pelvis, torso, head, arms, end effectors, and sensors. Maintaining this hierarchy allows navigation-scale localization and manipulation-scale perception to coexist without requiring identical spatial resolution.

Semantic perception can transform a purely geometric SLAM map into a task-relevant world representation. Doors, stairs, elevators, tables, shelves, tools, machines, charging stations, people, and restricted regions can be associated with spatial locations. A humanoid can then reason about where actions should occur rather than treating the environment only as points, surfaces, and free space.

Dynamic environments require special handling because humanoids are expected to operate near people. Visual features belonging to walking humans, moving carts, opening doors, or manipulated objects should not be treated as permanent landmarks. Motion segmentation, semantic masks, object tracking, and robust estimation can reduce the influence of dynamic observations while preserving stable background structures for localization.

Lighting variation is another major challenge for visual SLAM. Industrial facilities and buildings may contain bright windows, dark corridors, reflective floors, flickering illumination, or transitions between indoor and outdoor areas. Exposure control, high-dynamic-range imaging, robust feature descriptors, image normalization, and complementary depth or inertial information can help maintain localization through these changes.

Localization uncertainty should influence humanoid behavior directly. If visual tracking quality decreases or inertial uncertainty grows, the robot can slow its walking speed, widen its stability margin, stop in a balanced stance, rotate its head to search for landmarks, or move toward a better-observed area. This prevents an estimation problem from becoming a balance, collision, or manipulation failure.

Relocalization is essential after tracking loss or system restart. A humanoid may compare current visual observations with previously stored keyframes or landmarks to recover its position within an existing map. Semantic landmarks can provide additional cues when geometric appearance is ambiguous. Once a reliable global pose is recovered, navigation and manipulation tasks can resume without rebuilding the entire environment.

Computational architecture must support multiple perception rates. IMU processing may operate at hundreds of measurements per second, visual tracking at camera frame rate, and global optimization at a slower frequency. Local estimation should remain responsive enough for balance and navigation, while background optimization improves global map consistency without blocking time-critical control and safety processes.

Production monitoring should include visual tracking quality, number and distribution of tracked features, reprojection error, IMU bias, pose covariance, loop closures, relocalization events, camera exposure status, transform latency, and foot-contact confidence. Correlating these indicators with walking stability and navigation performance can reveal localization degradation before it causes a mission-level failure.

Validation should reproduce complete humanoid operating conditions. Tests should include walking and turning, stair ascent and descent, crouching, head scanning, object manipulation, low-texture corridors, changing illumination, dynamic crowds, reflective surfaces, temporary occlusion, tracking loss, and restart-based relocalization. Repeated missions are necessary to evaluate both accuracy and operational repeatability.

Multiple humanoids can share a validated global map while maintaining independent local visual-inertial states. Fleet-level systems may distribute maps, assign missions, coordinate shared spaces, and monitor localization health. Changes detected by individual robots can be compared and validated before updating the common map, preventing transient objects or temporary environmental conditions from corrupting the fleet reference.

A mature humanoid visual-inertial SLAM architecture therefore combines cameras, inertial sensing, kinematics, contact estimation, depth perception, semantic understanding, and uncertainty-aware behavior. SLAM provides the spatial foundation, but dependable humanoid autonomy emerges when localization remains connected to balance, locomotion, active perception, manipulation, safety, and mission-level reasoning throughout the complete perception-to-action loop.

휴머노이드 로봇(Humanoid Robot)은 보행, 방향 전환, 계단 이동, 객체 조작, 그리고 사람을 위해 설계된 환경과의 상호작용 과정에서 지속적인 위치추정(Localization)이 필요하다. 시각-관성 SLAM(Visual-Inertial SLAM)은 카메라가 풍부한 기하학적·의미론적 관측 정보를 제공하고 관성측정장치(IMU)가 빠른 몸체 움직임을 측정하기 때문에 특히 적합하다. 두 센서를 결합하면 외부 인프라에 지속적으로 의존하지 않고 6자유도 포즈(6-DoF Pose)를 추정할 수 있다.

일반적인 휴머노이드 인식 시스템(Humanoid Perception System)은 스테레오 또는 RGB-D 카메라, 광각 내비게이션 카메라(Wide-angle Navigation Camera), IMU, 관절 엔코더(Joint Encoder), 발 접촉 센싱(Foot-contact Sensing)을 포함한다. 카메라는 기능에 따라 머리, 몸통 또는 손목에 장착할 수 있다. 머리 카메라는 장거리 환경 인식을 제공하고 손목 카메라는 조작 작업을 지원한다. IMU는 동적인 몸체 움직임 동안 고주파 각속도와 가속도 정보를 제공한다.

휴머노이드 운동학(Humanoid Kinematics)은 바퀴형 로봇과 다른 위치추정 문제를 발생시킨다. 골반(Pelvis)이 움직이고 몸통(Torso)이 균형을 보상하며 머리가 목표물을 추적하기 때문에 센서 플랫폼은 지속적으로 병진 및 회전 운동을 수행한다. 보행은 주기적인 수직 변위와 진동을 발생시킨다. 시각-관성 상태 추정(Visual-Inertial Estimation)은 이러한 실제 몸체 움직임과 센서 노이즈를 구분하면서 로봇의 위치와 자세에 대한 안정적인 추정값을 유지해야 한다.

카메라와 IMU 사이의 정확한 보정(Calibration)은 필수적이다. 외부 파라미터 보정(Extrinsic Calibration)은 두 센서 사이의 상대적인 위치와 방향을 결정하고, 시간 보정(Temporal Calibration)은 이미지 노출 시점과 관성 측정값을 정렬한다. 빠른 머리 회전이나 보행 중에는 작은 시간 오차도 상당한 영향을 줄 수 있다. 안정적인 시각 추적(Visual Tracking)을 위해 카메라 내부 파라미터(Camera Intrinsic Parameter), 렌즈 왜곡(Lens Distortion), 스테레오 기하(Stereo Geometry), 롤링 셔터(Rolling Shutter) 특성도 정확하게 파악해야 한다.

시각-관성 오도메트리(Visual-Inertial Odometry)는 서로 보완적인 센싱 특성을 결합한다. IMU는 높은 주파수로 단기 움직임을 예측하지만 바이어스(Bias)와 드리프트(Drift)가 누적되며, 시각 관측은 연속 영상에서 특징점을 추적하여 기하학적 제약조건을 제공한다. 최적화 과정은 로봇 움직임, 랜드마크(Landmark), 속도, 관성 바이어스를 함께 추정하여 시각 정보로 관성 드리프트를 보정하고 관성 예측을 이용하여 빠른 움직임에서도 시각 추적을 안정화한다.

특징 기반 시각 SLAM(Feature-based Visual SLAM)은 코너(Corner), 에지(Edge), 기타 특징적인 영상 구조를 검출하고 프레임 사이의 대응 관계를 생성할 수 있다. 충분한 텍스처가 존재하는 환경에서는 이러한 대응 관계가 카메라 움직임을 제한하고 지도 작성을 지원한다. 직접법(Direct Method) 또는 준직접법(Semi-direct Method)은 영상 밝기 정보(Image Intensity)를 추가로 활용하여 명시적인 특징 추출만으로 충분하지 않은 환경에서도 유용한 상태 추정값을 제공할 수 있다.

깊이 센싱(Depth Sensing)은 휴머노이드 주변의 기하학적 환경 이해를 향상시킨다. 스테레오 비전(Stereo Vision)은 영상 시차(Image Disparity)를 이용하여 깊이를 추정하고, RGB-D 카메라는 동작 범위 내에서 직접적인 깊이 측정값을 제공한다. 이러한 관측으로 포인트 클라우드(Point Cloud), 복셀 지도(Voxel Map), 점유 모델(Occupancy Model), 표면 표현(Surface Representation)을 생성하여 내비게이션, 충돌 회피, 계단 검출, 조작 및 인간 규모 환경과의 상호작용을 지원할 수 있다.

장시간 운용에서는 루프 폐쇄(Loop Closure)가 필요하다. 휴머노이드가 이전에 관측했던 방, 복도, 작업 스테이션 또는 건물 영역으로 다시 돌아오면 시각적 장소 인식(Visual Place Recognition)을 이용하여 해당 위치를 식별하고 루프 제약조건(Loop Constraint)을 생성할 수 있다. 이후 포즈 그래프(Pose Graph) 또는 번들 조정(Bundle Adjustment) 최적화를 통해 누적된 궤적 드리프트를 감소시키고 서로 다른 시점에 관측된 공간 영역 사이의 일관성을 향상시킨다.

다리 운동학(Leg Kinematics)은 추가적인 움직임 제약조건을 제공할 수 있다. 안정적인 지지 단계(Stance Phase)에서는 지지 발(Support Foot)을 일시적으로 지면에 대한 기준으로 사용할 수 있다. 관절 엔코더 측정값과 로봇 운동학 모델을 이용하면 해당 접촉점을 기준으로 몸체 움직임을 추정할 수 있다. 이러한 정보를 시각-관성 상태 추정과 결합하면 중복성(Redundancy)이 증가하고 시각 추적이 일시적으로 약해지는 상황에서도 위치추정 성능을 향상시킬 수 있다.

그러나 발 접촉 정보(Foot-contact Information)는 확률적으로 처리해야 한다. 휴머노이드는 미끄러운 바닥, 카펫, 경사로, 계단, 잔해 또는 변형 가능한 표면 위를 걸을 수 있으며, 이러한 환경에서는 지지 발이 완전히 고정되지 않을 수 있다. 힘-토크 센서(Force-torque Sensor), 압력 센서(Pressure Sensor), 관절 토크 추정(Joint Torque Estimation), 접촉 상태 추정(Contact-state Estimation)을 이용하여 발 접촉 제약조건을 위치추정에 얼마나 강하게 반영할지 판단할 수 있다.

머리 움직임(Head Motion)은 기회인 동시에 도전 요소이다. AMR에 고정된 카메라와 달리 휴머노이드는 유용한 랜드마크를 관측하거나 불확실한 영역을 검사하거나 시각 추적을 복구하기 위해 의도적으로 머리를 회전시킬 수 있다. 능동 인식(Active Perception)은 특징 가시성을 높이거나 불확실성을 감소시키는 관측 방향을 선택할 수 있다. 그러나 위치추정 시스템은 움직이는 머리와 로봇 몸체 사이의 운동학적 변환(Kinematic Transformation)을 정확하게 반영해야 한다.

조작(Manipulation)은 추가적인 좌표 관계를 발생시킨다. 로봇은 머리 카메라를 이용하여 전역 위치를 추정하면서 손목 카메라를 통해 손에 대한 객체의 상대 포즈를 추정할 수 있다. 일관된 좌표 변환 계층(Transform Hierarchy)은 전역 지도(Global Map), 골반, 몸통, 머리, 팔, 엔드 이펙터(End Effector), 센서를 연결한다. 이러한 구조를 유지하면 내비게이션 규모의 위치추정과 조작 규모의 인식이 동일한 공간 해상도를 요구하지 않으면서 공존할 수 있다.

의미론적 인식(Semantic Perception)은 순수한 기하학적 SLAM 지도를 작업과 관련된 세계 표현(Task-relevant World Representation)으로 확장할 수 있다. 문, 계단, 엘리베이터, 테이블, 선반, 공구, 기계, 충전소, 사람, 제한 구역 등의 정보를 공간 위치와 연결할 수 있다. 이를 통해 휴머노이드는 환경을 단순한 점, 표면, 자유 공간으로만 처리하는 것이 아니라 특정 행동을 어디에서 수행해야 하는지 추론할 수 있다.

동적 환경(Dynamic Environment)은 휴머노이드가 사람 주변에서 운용될 가능성이 높기 때문에 특별한 처리가 필요하다. 이동하는 사람, 카트, 열리고 닫히는 문, 조작되는 객체에 포함된 시각 특징을 영구적인 랜드마크로 처리해서는 안 된다. 움직임 분할(Motion Segmentation), 의미론적 마스크(Semantic Mask), 객체 추적(Object Tracking), 강건한 상태 추정(Robust Estimation)을 이용하여 동적 관측값의 영향을 줄이면서 안정적인 배경 구조를 위치추정에 활용할 수 있다.

조명 변화(Lighting Variation)도 시각 SLAM의 주요 문제이다. 산업 시설과 건물 내부에는 밝은 창문, 어두운 복도, 반사 바닥, 깜박이는 조명, 실내와 실외 사이의 급격한 조명 변화가 존재할 수 있다. 노출 제어(Exposure Control), 고명암비 영상(High-dynamic-range Imaging), 강건한 특징 기술자(Robust Feature Descriptor), 영상 정규화(Image Normalization), 보완적인 깊이 또는 관성 정보를 이용하여 이러한 변화에서도 위치추정을 유지할 수 있다.

위치추정 불확실성(Localization Uncertainty)은 휴머노이드 행동에 직접 반영되어야 한다. 시각 추적 품질이 감소하거나 관성 불확실성이 증가하면 로봇은 보행 속도를 낮추고, 안정성 여유(Stability Margin)를 확대하며, 균형 잡힌 자세로 정지하거나, 랜드마크를 탐색하기 위해 머리를 회전하거나, 관측 조건이 더 좋은 영역으로 이동할 수 있다. 이를 통해 상태 추정 문제가 균형 상실, 충돌 또는 조작 실패로 확대되는 것을 방지할 수 있다.

추적 손실(Tracking Loss)이나 시스템 재시작 이후에는 재위치추정(Relocalization)이 필수적이다. 휴머노이드는 현재의 시각 관측값을 이전에 저장된 키프레임(Keyframe) 또는 랜드마크와 비교하여 기존 지도에서 자신의 위치를 복구할 수 있다. 기하학적 외형이 모호한 경우 의미론적 랜드마크가 추가적인 단서를 제공할 수 있다. 신뢰할 수 있는 전역 포즈가 복구되면 전체 환경을 다시 지도화하지 않고 내비게이션과 조작 작업을 재개할 수 있다.

컴퓨팅 아키텍처(Computing Architecture)는 서로 다른 인식 처리 주기를 지원해야 한다. IMU 처리는 초당 수백 회 수준으로 동작할 수 있으며, 시각 추적은 카메라 프레임 속도로 수행되고, 전역 최적화(Global Optimization)는 상대적으로 낮은 주기로 실행될 수 있다. 지역 상태 추정(Local Estimation)은 균형 제어와 내비게이션에 충분히 빠르게 반응해야 하며, 백그라운드 최적화는 시간 결정적인 제어 및 안전 프로세스를 방해하지 않으면서 전역 지도 일관성을 향상시켜야 한다.

생산 환경 모니터링(Production Monitoring)에서는 시각 추적 품질, 추적 특징점의 수와 분포, 재투영 오차(Reprojection Error), IMU 바이어스, 포즈 공분산(Pose Covariance), 루프 폐쇄, 재위치추정 이벤트, 카메라 노출 상태, 좌표 변환 지연시간(Transform Latency), 발 접촉 신뢰도 등을 관리해야 한다. 이러한 지표를 보행 안정성과 내비게이션 성능에 연계하면 임무 수준의 장애가 발생하기 전에 위치추정 성능 저하를 식별할 수 있다.

검증(Validation)은 휴머노이드의 전체 운용 조건을 재현해야 한다. 보행과 방향 전환, 계단 상승 및 하강, 몸 낮추기(Crouching), 머리 스캐닝(Head Scanning), 객체 조작, 텍스처가 부족한 복도, 조명 변화, 동적 군중, 반사 표면, 일시적인 가림, 추적 손실, 시스템 재시작 이후 재위치추정 등을 시험해야 한다. 정확도뿐만 아니라 실제 운용의 반복성(Operational Repeatability)을 평가하기 위해 반복적인 임무 시험이 필요하다.

여러 휴머노이드는 검증된 전역 지도(Global Map)를 공유하면서 각각 독립적인 지역 시각-관성 상태(Local Visual-Inertial State)를 유지할 수 있다. 플릿 수준 시스템(Fleet-level System)은 지도를 배포하고 임무를 할당하며 공유 공간을 조정하고 위치추정 상태를 모니터링할 수 있다. 개별 로봇이 발견한 환경 변화는 공통 지도를 갱신하기 전에 비교 및 검증하여 일시적인 객체나 환경 조건이 플릿 기준 지도를 손상시키는 것을 방지할 수 있다.

성숙한 휴머노이드 시각-관성 SLAM 아키텍처(Humanoid Visual-Inertial SLAM Architecture)는 카메라, 관성 센싱(Inertial Sensing), 운동학(Kinematics), 접촉 상태 추정(Contact Estimation), 깊이 인식(Depth Perception), 의미론적 이해(Semantic Understanding), 불확실성 기반 행동(Uncertainty-aware Behavior)을 통합한다. SLAM은 공간 인식의 기반을 제공하지만, 신뢰할 수 있는 휴머노이드 자율성(Humanoid Autonomy)은 위치추정이 균형 제어, 보행, 능동 인식, 조작, 안전, 임무 수준 추론과 전체 인식-행동 루프(Perception-to-action Loop)에서 지속적으로 연결될 때 구현될 수 있다.

##  

## 12.06. Cargo UAV GPS Denied LiDAR SLAM Case

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

Cargo UAVs operating in GPS-denied environments require localization that remains reliable when satellite navigation becomes unavailable, degraded, jammed, or obstructed. Warehouses, tunnels, urban canyons, ports, mountainous corridors, industrial structures, and low-altitude flight near large buildings are representative cases. LiDAR SLAM provides an onboard geometric reference that allows the aircraft to estimate six-degree-of-freedom motion without continuous GNSS dependence.

A cargo UAV localization system typically combines 3D LiDAR, IMU, cameras, barometric altitude sensing, radar or range sensors, and GNSS when available. LiDAR provides geometric measurements of surrounding structures, while the IMU captures high-rate angular velocity and acceleration. Cameras can supplement geometric information, and independent altitude sensors provide additional constraints during takeoff, landing, and low-altitude flight.

Unlike ground robots, UAVs move freely in three dimensions and cannot rely on wheel odometry or persistent ground contact. Translation, rotation, vertical motion, acceleration, and aerodynamic disturbance can occur simultaneously. The SLAM estimator must therefore maintain full 6-DoF state estimation while remaining responsive enough to provide localization information to the flight controller during dynamic maneuvers.

Accurate sensor calibration is fundamental to this architecture. The rigid transformation between LiDAR and IMU must be known precisely, and timestamps must be synchronized sufficiently well to compensate for aircraft motion during each LiDAR scan. Calibration errors or temporal offsets can distort the point cloud and introduce systematic pose errors, particularly during rapid yaw changes, acceleration, vibration, or turbulent flight.

LiDAR-inertial odometry combines high-frequency inertial prediction with geometric correction. IMU measurements propagate the vehicle state between LiDAR observations, while scan registration aligns current geometric measurements with previous scans or a local map. This complementary relationship limits inertial drift while preserving fast state updates, making LiDAR-inertial estimation particularly useful when GNSS disappears unexpectedly.

Motion compensation is essential because a UAV can move significantly during a single LiDAR scan. Raw points acquired at different times represent different sensor poses and should not be treated as if they were captured simultaneously. IMU-based deskewing transforms measurements toward a common temporal reference, improving geometric consistency and increasing the reliability of scan matching during aggressive or disturbed flight.

The map representation must support three-dimensional flight rather than planar navigation. Point clouds, voxel maps, occupancy grids, surfel representations, or distance fields can describe walls, ceilings, structural beams, containers, vegetation, terrain, and other obstacles. The localization map can also provide geometric information for collision avoidance and route planning, allowing perception and navigation to share a consistent spatial representation.

GPS-denied flight often begins as a transition rather than a completely separate mission mode. A cargo UAV may initially navigate with GNSS or RTK-GNSS and then enter a warehouse, tunnel, covered logistics facility, or region affected by satellite blockage. The localization architecture should transfer smoothly from globally referenced navigation to LiDAR-inertial estimation without introducing sudden position or orientation discontinuities.

When GNSS becomes available again, global measurements should be reintegrated carefully. A direct position jump can destabilize navigation or produce an unsafe trajectory correction. Instead, the system can compare the SLAM trajectory with the recovered global reference, evaluate measurement integrity, and gradually reconcile accumulated drift through filtering or graph optimization while maintaining continuous control coordinates.

Loop closure helps control long-term drift during extended GPS-denied missions. When the UAV revisits a previously mapped structure, corridor, loading area, or waypoint region, geometric place recognition can identify the overlap. A loop constraint can then be introduced into the pose graph, allowing optimization to redistribute accumulated error and improve consistency across the complete flight trajectory.

Degenerate geometry is a significant challenge for airborne LiDAR SLAM. Long tunnels, large flat walls, open warehouses, repetitive container rows, or high-altitude areas with limited nearby structure may provide insufficient geometric constraints in one or more directions. The estimator should detect such observability loss rather than reporting unrealistically high confidence in a weakly constrained pose solution.

Sensor fusion can reduce this vulnerability. Visual features may provide constraints where LiDAR geometry is repetitive, while radar can remain useful in dust, fog, or adverse visibility. Barometric altitude, downward range sensing, and inertial measurements can provide additional vertical information. The objective is not simply to add sensors, but to ensure that their failure characteristics are sufficiently different to provide meaningful redundancy.

Cargo payload introduces additional localization challenges. Changes in payload mass and center of gravity affect acceleration response, vibration, attitude control, and structural dynamics. Suspended loads can generate oscillatory motion that differs substantially from rigid-body flight. Although SLAM estimates spatial motion rather than vehicle dynamics directly, these effects influence sensor stability and should be represented in validation and estimator tuning.

Vibration management is particularly important on large UAVs. Propellers, motors, gearboxes, structural modes, and payload motion can contaminate inertial measurements and mechanically disturb LiDAR or camera alignment. Rigid sensor mounting, characterized vibration isolation, appropriate IMU filtering, structural monitoring, and calibration verification are required so that vibration mitigation does not introduce unknown sensor motion or excessive latency.

The local map should remain computationally bounded during long missions. Maintaining every historical LiDAR point onboard is unnecessary and can increase memory and optimization cost. Keyframe selection, voxel filtering, sliding local maps, submaps, and hierarchical representations can preserve useful geometry while limiting processing requirements. Older map regions can remain available for loop closure without participating continuously in local scan registration.

Localization uncertainty must influence flight behavior directly. When geometric constraints weaken or sensor consistency decreases, the UAV can reduce speed, increase obstacle clearance, avoid narrow passages, hold position when feasible, or move toward a region with stronger observable structure. A localization system should therefore provide confidence information to navigation and flight management rather than only outputting a nominal pose.

Failure detection should distinguish temporary degradation from complete localization loss. Rising scan-matching residuals, increasing covariance, abnormal IMU bias, inconsistent sensor innovations, insufficient geometric features, or repeated registration failures can indicate deteriorating estimation. Monitoring these indicators allows the autonomy system to initiate recovery before pose error becomes large enough to threaten flight safety.

Recovery behavior depends on the available environment and remaining state confidence. The UAV may hover, reduce motion, rotate to acquire additional geometry, backtrack along a recently validated corridor, ascend or descend toward more observable structures, or switch to an alternative localization modality. If reliable localization cannot be restored, mission logic should transition toward a predefined contingency rather than continuing normal autonomous flight.

Takeoff and landing require special consideration because proximity to the ground and surrounding infrastructure changes the sensing geometry rapidly. Downward LiDAR, radar altimeters, cameras, or range sensors can provide complementary height and surface information. Landing-zone geometry should be validated independently from global position estimates so that a locally consistent SLAM solution can support precise terminal guidance even without GNSS.

Map semantics can extend the SLAM representation beyond geometry. Loading zones, landing areas, restricted volumes, tunnel entrances, structural hazards, emergency holding regions, and known communication-loss zones can be associated with mapped coordinates. This allows the autonomy system to reason about mission constraints while operating within the same spatial reference used for localization and obstacle avoidance.

Production monitoring should include LiDAR registration quality, IMU bias, pose covariance, map overlap, loop-closure status, processing latency, vibration indicators, GNSS integrity, sensor synchronization, and localization-mode transitions. Recording these variables together with flight-control data makes it possible to distinguish estimation failures from aerodynamic, mechanical, communication, or control problems during post-flight analysis.

Validation should deliberately reproduce GNSS degradation rather than treating it as an exceptional event. Flight tests should include controlled GNSS loss and recovery, tunnels, repetitive structures, sparse geometry, vibration, dynamic obstacles, varying payloads, altitude changes, aggressive turns, and transitions between indoor and outdoor environments. Repeated missions are necessary to characterize drift, recovery behavior, and operational repeatability.

For cargo UAVs, localization must ultimately be evaluated as part of the complete flight-safety architecture. LiDAR SLAM, inertial estimation, alternative sensing, navigation, flight control, health monitoring, and contingency management must exchange consistent state and confidence information. A technically accurate SLAM trajectory alone is insufficient if degradation cannot be detected or communicated to the systems responsible for safe vehicle behavior.

A mature GPS-denied cargo UAV architecture therefore treats LiDAR SLAM as a resilient navigation component rather than a replacement for GNSS. Global navigation provides absolute reference when trustworthy, while LiDAR-inertial SLAM maintains locally consistent autonomy when that reference disappears. Sensor redundancy, uncertainty awareness, recovery logic, and flight-level safety integration together enable dependable operation across transitions between globally referenced and GPS-denied environments.

GPS 음영 환경(GPS-denied Environment)에서 운용되는 화물 무인항공기(Cargo UAV)는 위성항법이 사용할 수 없거나, 성능이 저하되거나, 재밍(Jamming)을 받거나, 장애물에 의해 차단되는 상황에서도 신뢰할 수 있는 위치추정(Localization)을 유지해야 한다. 창고, 터널, 도심 협곡(Urban Canyon), 항만, 산악 통로, 산업 구조물, 대형 건물 인근의 저고도 비행 등이 대표적인 사례이다. 라이다 SLAM(LiDAR SLAM)은 지속적인 GNSS 의존 없이 기체의 6자유도 운동(6-DoF Motion)을 추정할 수 있는 온보드 기하학적 기준(Onboard Geometric Reference)을 제공한다.

화물 UAV 위치추정 시스템(Cargo UAV Localization System)은 일반적으로 3D 라이다(3D LiDAR), 관성측정장치(IMU), 카메라, 기압 고도 센서(Barometric Altitude Sensor), 레이더 또는 거리 센서(Range Sensor), 그리고 사용 가능한 경우 GNSS를 결합한다. 라이다는 주변 구조물의 기하학적 측정값을 제공하고 IMU는 고주파 각속도와 가속도를 측정한다. 카메라는 기하학적 정보를 보완하며, 독립적인 고도 센서는 이륙, 착륙 및 저고도 비행에서 추가적인 제약조건을 제공한다.

지상 로봇과 달리 UAV는 3차원 공간에서 자유롭게 움직이며 휠 오도메트리(Wheel Odometry)나 지속적인 지면 접촉에 의존할 수 없다. 병진, 회전, 수직 운동, 가속, 공기역학적 외란(Aerodynamic Disturbance)이 동시에 발생할 수 있다. 따라서 SLAM 상태 추정기(Estimator)는 완전한 6자유도 상태 추정(6-DoF State Estimation)을 유지하면서 동적 기동 과정에서도 비행 제어기(Flight Controller)에 충분히 빠르게 위치 정보를 제공해야 한다.

정확한 센서 보정(Sensor Calibration)은 이러한 아키텍처의 기본 요소이다. 라이다와 IMU 사이의 강체 변환(Rigid Transformation)을 정밀하게 파악해야 하며, 각각의 라이다 스캔에서 발생하는 기체 움직임을 보정할 수 있도록 타임스탬프(Timestamp)를 충분한 정확도로 동기화해야 한다. 보정 오차나 시간 오프셋(Temporal Offset)은 포인트 클라우드(Point Cloud)를 왜곡하고 체계적인 포즈 오차를 발생시킬 수 있으며, 빠른 요(Yaw) 변화, 가속, 진동 또는 난류 비행에서 이러한 영향은 더욱 커진다.

라이다-관성 오도메트리(LiDAR-Inertial Odometry)는 고주파 관성 예측(Inertial Prediction)과 기하학적 보정(Geometric Correction)을 결합한다. IMU 측정값은 라이다 관측 사이에서 기체 상태를 전파하고, 스캔 정합(Scan Registration)은 현재의 기하학적 측정값을 이전 스캔 또는 지역 지도(Local Map)에 정합한다. 이러한 상호 보완 관계는 빠른 상태 갱신을 유지하면서 관성 드리프트(Inertial Drift)를 제한하므로 GNSS가 갑자기 사라지는 상황에서 특히 유용하다.

UAV는 하나의 라이다 스캔이 완료되는 동안에도 상당한 거리를 이동할 수 있으므로 모션 보정(Motion Compensation)이 필수적이다. 서로 다른 시간에 획득된 원시 포인트(Raw Point)는 서로 다른 센서 포즈를 나타내므로 동시에 측정된 것처럼 처리해서는 안 된다. IMU 기반 디스큐(IMU-based Deskewing)는 측정값을 공통 시간 기준(Common Temporal Reference)으로 변환하여 기하학적 일관성을 향상시키고, 공격적인 기동이나 외란이 발생하는 비행에서도 스캔 정합의 신뢰성을 높인다.

지도 표현(Map Representation)은 평면 내비게이션이 아니라 3차원 비행을 지원해야 한다. 포인트 클라우드, 복셀 지도(Voxel Map), 점유 격자(Occupancy Grid), 서펠 표현(Surfel Representation), 거리장(Distance Field)을 이용하여 벽, 천장, 구조용 빔, 컨테이너, 식생, 지형 및 기타 장애물을 표현할 수 있다. 위치추정 지도는 충돌 회피와 경로 계획에 필요한 기하학적 정보도 제공할 수 있으므로 인식과 내비게이션이 일관된 공간 표현을 공유할 수 있다.

GPS 음영 비행은 완전히 독립적인 임무 모드로 시작되기보다 운용 중 전환(Transition) 형태로 발생하는 경우가 많다. 화물 UAV는 처음에는 GNSS 또는 RTK-GNSS를 이용하여 항법하다가 창고, 터널, 지붕이 있는 물류 시설 또는 위성 신호가 차단되는 영역으로 진입할 수 있다. 위치추정 아키텍처는 갑작스러운 위치나 자세 불연속을 발생시키지 않으면서 전역 기준 항법(Global Referenced Navigation)에서 라이다-관성 상태 추정으로 안정적으로 전환할 수 있어야 한다.

GNSS가 다시 사용 가능해지는 경우에도 전역 측정값(Global Measurement)은 신중하게 재통합해야 한다. 위치를 직접적으로 급격히 변경하면 내비게이션이 불안정해지거나 위험한 궤적 보정이 발생할 수 있다. 대신 시스템은 SLAM 궤적과 복구된 전역 기준을 비교하고 측정 무결성(Measurement Integrity)을 평가한 후, 연속적인 제어 좌표계를 유지하면서 필터링 또는 그래프 최적화(Graph Optimization)를 통해 누적된 드리프트를 점진적으로 보정할 수 있다.

장시간 GPS 음영 임무에서는 루프 폐쇄(Loop Closure)가 장기적인 드리프트를 제어하는 데 도움을 준다. UAV가 이전에 지도화한 구조물, 통로, 적재 구역 또는 웨이포인트 영역(Waypoint Region)을 다시 방문하면 기하학적 장소 인식(Geometric Place Recognition)을 통해 중첩 영역을 식별할 수 있다. 이후 포즈 그래프(Pose Graph)에 루프 제약조건을 추가하고 최적화를 수행하여 누적 오차를 전체 비행 궤적에 재분배하고 일관성을 향상시킬 수 있다.

퇴화된 기하학 구조(Degenerate Geometry)는 공중 라이다 SLAM에서 중요한 문제이다. 긴 터널, 대형 평면 벽, 개방형 창고, 반복되는 컨테이너 열 또는 주변 구조물이 부족한 고고도 영역에서는 하나 이상의 방향에서 충분한 기하학적 제약조건을 확보하지 못할 수 있다. 상태 추정기는 이러한 관측 가능성 손실(Observability Loss)을 검출하여 제약이 약한 포즈 추정값에 비현실적으로 높은 신뢰도를 부여하지 않아야 한다.

센서 융합(Sensor Fusion)은 이러한 취약성을 감소시킬 수 있다. 라이다 기하학이 반복적인 환경에서는 시각 특징(Visual Feature)이 추가적인 제약조건을 제공할 수 있으며, 레이더(Radar)는 먼지, 안개 또는 불리한 가시 조건에서도 유용할 수 있다. 기압 고도, 하향 거리 센싱(Downward Range Sensing), 관성 측정값도 추가적인 수직 방향 정보를 제공할 수 있다. 중요한 것은 단순히 센서 수를 늘리는 것이 아니라 서로 다른 고장 특성을 가진 센서를 조합하여 의미 있는 중복성(Redundancy)을 확보하는 것이다.

화물 탑재(Payload)는 추가적인 위치추정 문제를 발생시킨다. 탑재 질량과 무게중심(Center of Gravity)의 변화는 가속 응답, 진동, 자세 제어, 구조 동역학(Structural Dynamics)에 영향을 준다. 매달린 화물(Suspended Load)은 강체 비행과 상당히 다른 진동 운동을 발생시킬 수 있다. SLAM 자체는 차량 동역학이 아니라 공간 움직임을 추정하지만, 이러한 영향은 센서 안정성에 직접 영향을 주므로 검증 및 상태 추정기 튜닝 과정에서 고려해야 한다.

대형 UAV에서는 진동 관리(Vibration Management)가 특히 중요하다. 프로펠러, 모터, 기어박스, 구조 진동 모드(Structural Mode), 화물 움직임은 관성 측정값에 노이즈를 발생시키고 라이다 또는 카메라 정렬을 기계적으로 교란할 수 있다. 견고한 센서 장착, 특성이 검증된 진동 절연(Vibration Isolation), 적절한 IMU 필터링, 구조 상태 모니터링, 보정 검증을 적용하여 진동 완화 장치 자체가 알 수 없는 센서 움직임이나 과도한 지연시간을 발생시키지 않도록 해야 한다.

장시간 임무에서도 지역 지도(Local Map)의 연산량은 제한된 범위로 유지해야 한다. 모든 과거 라이다 포인트를 온보드에서 계속 유지할 필요는 없으며, 이는 메모리와 최적화 비용을 증가시킨다. 키프레임 선택(Keyframe Selection), 복셀 필터링(Voxel Filtering), 슬라이딩 지역 지도(Sliding Local Map), 서브맵(Submap), 계층적 표현(Hierarchical Representation)을 이용하면 유용한 기하학 정보를 보존하면서 처리 요구량을 제한할 수 있다. 오래된 지도 영역은 지역 스캔 정합에 계속 참여하지 않더라도 루프 폐쇄를 위해 유지할 수 있다.

위치추정 불확실성(Localization Uncertainty)은 비행 행동에 직접 반영되어야 한다. 기하학적 제약조건이 약해지거나 센서 사이의 일관성이 감소하면 UAV는 속도를 줄이고, 장애물과의 여유 거리(Obstacle Clearance)를 증가시키며, 좁은 통로를 피하고, 가능한 경우 위치를 유지하거나 관측 가능한 구조물이 더 풍부한 영역으로 이동할 수 있다. 따라서 위치추정 시스템은 명목상의 포즈만 출력하는 것이 아니라 내비게이션과 비행 관리 시스템에 신뢰도 정보를 함께 제공해야 한다.

고장 검출(Failure Detection)은 일시적인 성능 저하와 완전한 위치추정 손실을 구분할 수 있어야 한다. 증가하는 스캔 정합 잔차(Scan-matching Residual), 공분산(Covariance) 증가, 비정상적인 IMU 바이어스, 센서 혁신값(Innovation)의 불일치, 부족한 기하학적 특징 또는 반복적인 정합 실패는 상태 추정 성능이 악화되고 있음을 나타낼 수 있다. 이러한 지표를 모니터링하면 포즈 오차가 비행 안전을 위협할 정도로 증가하기 전에 자율 시스템이 복구 절차를 시작할 수 있다.

복구 행동(Recovery Behavior)은 주변 환경과 남아 있는 상태 추정 신뢰도에 따라 달라진다. UAV는 호버링(Hovering)을 수행하거나 움직임을 줄이고, 추가적인 기하학 구조를 획득하기 위해 회전하거나, 최근 검증된 통로를 따라 후퇴하거나, 관측 가능한 구조물이 더 많은 고도로 상승 또는 하강하거나, 대체 위치추정 방식(Alternative Localization Modality)으로 전환할 수 있다. 신뢰할 수 있는 위치추정을 복구하지 못한다면 정상적인 자율 비행을 계속하기보다 사전에 정의된 비상 대응(Contingency) 절차로 전환해야 한다.

이륙과 착륙(Takeoff and Landing)은 지면과 주변 인프라에 대한 거리가 빠르게 변하면서 센싱 기하학도 크게 변화하므로 특별한 고려가 필요하다. 하향 라이다(Downward LiDAR), 레이더 고도계(Radar Altimeter), 카메라 또는 거리 센서는 상호 보완적인 높이 및 표면 정보를 제공할 수 있다. 착륙 구역의 기하학적 정보는 전역 위치 추정값과 독립적으로 검증하여 GNSS가 없는 상황에서도 지역적으로 일관된 SLAM 결과를 기반으로 정밀한 종말 유도(Terminal Guidance)를 수행할 수 있도록 해야 한다.

지도 의미론(Map Semantics)은 SLAM 표현을 순수한 기하학 정보 이상으로 확장할 수 있다. 적재 구역(Loading Zone), 착륙 구역(Landing Area), 제한 공역(Restricted Volume), 터널 입구, 구조적 위험 요소, 비상 체공 영역(Emergency Holding Region), 알려진 통신 음영 구역(Communication-loss Zone)을 지도 좌표와 연결할 수 있다. 이를 통해 자율 시스템은 위치추정 및 장애물 회피에 사용하는 동일한 공간 기준 안에서 임무 제약조건을 함께 추론할 수 있다.

생산 환경 모니터링(Production Monitoring)에서는 라이다 정합 품질, IMU 바이어스, 포즈 공분산, 지도 중첩도(Map Overlap), 루프 폐쇄 상태, 처리 지연시간, 진동 지표, GNSS 무결성(GNSS Integrity), 센서 동기화 상태, 위치추정 모드 전환(Localization-mode Transition)을 관리해야 한다. 이러한 변수와 비행 제어 데이터를 함께 기록하면 비행 후 분석(Post-flight Analysis)에서 상태 추정 문제와 공기역학적·기계적·통신·제어 문제를 구분할 수 있다.

검증(Validation)에서는 GNSS 성능 저하를 예외적인 사건으로 취급하지 말고 의도적으로 재현해야 한다. 비행 시험에는 통제된 GNSS 손실 및 복구, 터널, 반복적인 구조물, 희소한 기하학 환경, 진동, 동적 장애물, 다양한 화물 조건, 고도 변화, 급격한 방향 전환, 실내와 실외 환경 사이의 전환을 포함해야 한다. 드리프트, 복구 동작, 운용 반복성(Operational Repeatability)을 특성화하기 위해 반복적인 임무 시험이 필요하다.

화물 UAV의 위치추정은 궁극적으로 전체 비행 안전 아키텍처(Flight-safety Architecture)의 일부로 평가해야 한다. 라이다 SLAM, 관성 상태 추정, 대체 센싱, 내비게이션, 비행 제어, 상태 모니터링(Health Monitoring), 비상 관리(Contingency Management)는 일관된 상태 및 신뢰도 정보를 교환해야 한다. 기술적으로 정확한 SLAM 궤적만으로는 충분하지 않으며, 성능 저하를 검출하고 안전한 기체 동작을 담당하는 시스템에 이를 전달할 수 있어야 한다.

성숙한 GPS 음영 화물 UAV 아키텍처(GPS-denied Cargo UAV Architecture)는 라이다 SLAM을 GNSS의 단순한 대체 수단이 아니라 복원력 있는 항법 구성요소(Resilient Navigation Component)로 취급한다. 전역 항법(Global Navigation)은 신뢰할 수 있을 때 절대 위치 기준을 제공하고, 라이다-관성 SLAM은 해당 기준이 사라질 때 지역적으로 일관된 자율성을 유지한다. 센서 중복성, 불확실성 인식(Uncertainty Awareness), 복구 로직, 비행 수준 안전 통합을 결합함으로써 전역 기준 환경과 GPS 음영 환경 사이를 전환하는 과정에서도 신뢰할 수 있는 운용이 가능해진다.

##  

## 12.07. Multi Robot Collaborative Mapping Warehouse Case

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

Multi-robot collaborative mapping allows a warehouse fleet to construct and maintain a shared spatial representation by combining observations collected by multiple autonomous robots. Instead of requiring one dedicated mapping robot to traverse the entire facility, AMRs can explore different aisles, storage zones, loading areas, and production interfaces simultaneously while contributing locally generated map information to a common mapping framework.

A typical warehouse fleet equips each robot with 2D or 3D LiDAR, wheel encoders, IMU, and optionally cameras or depth sensors. Every robot performs local localization and mapping using its own sensor observations while maintaining an estimate of its trajectory. Local SLAM must remain operational even when communication with other robots or the central infrastructure is temporarily unavailable.

Collaborative mapping introduces a fundamental coordinate-frame problem because each robot can initially create its map within an independent local reference frame. Before these maps can be combined, the system must estimate transformations between robot coordinate systems. Overlapping observations, known initialization locations, fiducial landmarks, GNSS where available, or infrastructure references can provide constraints that connect independently generated maps.

Map overlap detection determines whether two robots have observed the same physical region. In a warehouse, repeated rack structures and visually similar aisles make this difficult because geometrically similar observations may originate from different locations. Reliable overlap detection therefore requires sufficient spatial context and consistency checking before a cross-robot correspondence is accepted as a valid map relationship.

Cross-robot loop closure extends conventional loop closure beyond a single trajectory. When Robot A observes a location previously mapped by Robot B, the mapping system can establish a relative pose constraint between their trajectories. This constraint connects previously independent pose graphs and enables the optimization system to estimate a common spatial relationship between the robots and their local maps.

Incorrect cross-robot loop closures can severely distort a shared map. Warehouses frequently contain repeated shelves, identical columns, uniform corridors, and symmetric layouts that can produce false place recognition. Candidate matches should therefore be verified using geometric registration, consistency with neighboring observations, transformation plausibility, and other available sensor or semantic information before they influence global optimization.

Once valid inter-robot constraints are established, pose-graph optimization can jointly refine the trajectories of multiple robots. Odometry, local scan matching, individual loop closures, and cross-robot constraints are combined within a common optimization problem. The resulting solution reduces accumulated drift and aligns local maps into a globally consistent warehouse representation rather than simply overlaying point clouds using uncertain initial estimates.

The shared map can be represented as a 2D occupancy grid for conventional indoor AMRs or as a 3D point-cloud, voxel, or hybrid representation when vertical structure is important. Warehouses containing high racks, mezzanines, ramps, automated storage systems, or mobile manipulators may benefit from maintaining both a navigation-oriented 2D layer and a richer 3D structural map.

Map merging should distinguish geometric alignment from map-content fusion. Two maps may be correctly aligned but still contain conflicting occupancy information because they were observed at different times. Pallets, carts, forklifts, temporary barriers, and inventory can move between observations. The fusion process must therefore avoid treating every difference between robot maps as evidence of mapping error.

Persistent infrastructure should receive greater importance than temporary warehouse content. Walls, structural columns, fixed rack frames, doors, charging infrastructure, and permanently installed machines provide stable references for localization. Pallets, packages, human-operated vehicles, and temporary staging areas should instead be represented through dynamic or short-term layers where appropriate.

Communication constraints strongly influence collaborative mapping architecture. Continuously transmitting raw LiDAR scans from every robot can consume substantial bandwidth, particularly for large fleets. Robots can instead exchange compressed submaps, selected keyframes, descriptors, pose-graph constraints, or map changes. This reduces communication requirements while preserving information needed for collaborative alignment and optimization.

A centralized architecture can collect robot observations at a map server and perform cross-robot matching, global optimization, and map distribution in one infrastructure component. This simplifies global consistency management but creates dependence on network connectivity and server availability. The robots should therefore retain enough local autonomy to continue safe navigation during temporary communication interruptions.

A decentralized or distributed architecture allows robots to exchange mapping information directly or through multiple computational nodes. This can improve resilience and reduce dependence on one server, but synchronization, consistency, bandwidth allocation, and distributed optimization become more difficult. Hybrid architectures are often practical, combining independent local SLAM with infrastructure-assisted global map optimization.

Warehouse mapping can be accelerated by assigning complementary exploration regions to different robots. One AMR may map receiving docks while others cover storage aisles, production interfaces, or shipping zones. Exploration planning should minimize unnecessary duplication while deliberately preserving enough overlap between regions to create reliable cross-robot constraints required for map merging.

The collaborative map should be version controlled once it becomes an operational fleet asset. A candidate map generated from multiple robots can undergo geometric validation, route testing, docking verification, and semantic review before release. The approved version should remain available for rollback so that an incorrect map update does not immediately affect every robot in the warehouse.

Map updates require conflict-resolution policies. If one robot observes a blocked aisle while another recently observed the same location as free, the system must determine whether the difference represents a temporary obstacle or a structural change. Observation time, persistence, robot agreement, semantic classification, and confidence can contribute to deciding whether information belongs in the permanent map or a dynamic layer.

Semantic information can make collaborative mapping more useful for warehouse operations. Storage zones, loading docks, charging stations, elevators, restricted areas, intersections, one-way aisles, docking points, and production cells can be associated with geometric coordinates. Robots then share not only an occupancy representation but also a common operational interpretation of warehouse space.

Localization can benefit from collaborative observations even after initial mapping is complete. If several robots repeatedly experience poor scan matching in the same aisle, the fleet can identify a weak localization region. Another robot equipped with richer sensors may collect additional observations, or infrastructure landmarks can be introduced to improve the shared reference in that location.

Operational monitoring should track map alignment residuals, cross-robot loop closures, optimization consistency, map conflicts, communication latency, localization quality, map version, and data freshness. Fleet-level visualization can reveal whether individual robots are gradually diverging from the common map or whether a particular region is producing repeated inconsistencies across multiple platforms.

Failure isolation is essential because one malfunctioning robot should not corrupt the entire fleet map. Incorrect calibration, damaged LiDAR, wheel-slip-induced odometry errors, or faulty timestamps can produce systematically distorted local maps. Contributions from each robot should therefore carry quality information, allowing suspicious data to be rejected or quarantined before global map integration.

Recovery mechanisms should support both robot-level and map-level failures. A robot that loses localization can use the shared map for relocalization, while a corrupted global update can be rolled back to a previously validated map version. If communication is lost, robots can continue local navigation and synchronize accumulated mapping information after connectivity is restored.

Validation should reproduce fleet-scale warehouse operation rather than evaluating robots independently. Tests should include simultaneous mapping, overlapping and non-overlapping trajectories, repetitive rack aisles, communication loss, robot restart, temporary obstacles, structural changes, false loop-closure candidates, map-server interruption, and reintegration of robots returning after extended offline operation.

As fleet size increases, collaborative mapping becomes a data-management problem as much as a SLAM problem. Map observations, keyframes, robot trajectories, semantic annotations, timestamps, confidence values, and map versions must remain traceable. Efficient storage and selective distribution are necessary so that hundreds of robots do not continuously exchange information irrelevant to their current operating regions.

A mature warehouse collaborative mapping architecture therefore combines independent local SLAM, cross-robot place recognition, map merging, pose-graph optimization, communication management, conflict resolution, and controlled map distribution. The objective is not simply to combine maps from many robots, but to maintain a trustworthy shared spatial reference that improves localization, navigation, fleet coordination, and long-term warehouse operation.

다중 로봇 협업 지도작성(Multi-robot Collaborative Mapping)은 여러 자율 로봇이 수집한 관측 정보를 결합하여 창고 플릿(Warehouse Fleet)이 공유 공간 표현(Shared Spatial Representation)을 구축하고 유지할 수 있도록 한다. 하나의 전용 지도작성 로봇이 시설 전체를 이동하도록 하는 대신 여러 AMR이 서로 다른 통로, 저장 구역, 상하역 구역, 생산 인터페이스를 동시에 탐색하면서 각자 생성한 지역 지도 정보를 공통 지도작성 프레임워크(Common Mapping Framework)에 제공할 수 있다.

일반적인 창고 플릿은 각 로봇에 2D 또는 3D 라이다(LiDAR), 휠 엔코더(Wheel Encoder), 관성측정장치(IMU), 그리고 선택적으로 카메라 또는 깊이 센서(Depth Sensor)를 탑재한다. 각 로봇은 자체 센서 관측값을 이용하여 지역 위치추정 및 지도작성(Local Localization and Mapping)을 수행하면서 자신의 이동 궤적을 추정한다. 다른 로봇이나 중앙 인프라와의 통신이 일시적으로 단절되더라도 지역 SLAM(Local SLAM)은 계속 동작할 수 있어야 한다.

협업 지도작성은 각 로봇이 서로 독립적인 지역 기준 좌표계(Local Reference Frame)에서 지도를 생성할 수 있기 때문에 기본적인 좌표 프레임 문제(Coordinate-frame Problem)를 발생시킨다. 이러한 지도를 결합하려면 먼저 로봇 좌표계 사이의 변환(Transformation)을 추정해야 한다. 중첩된 관측값, 알려진 초기 위치, 기준 마커(Fiducial Landmark), 사용 가능한 경우 GNSS, 또는 인프라 기준점(Infrastructure Reference)을 이용하여 독립적으로 생성된 지도들을 연결하는 제약조건을 구성할 수 있다.

지도 중첩 검출(Map Overlap Detection)은 두 로봇이 동일한 물리적 영역을 관측했는지를 판단한다. 창고에서는 반복적인 랙 구조와 시각적으로 유사한 통로 때문에 서로 다른 위치에서 얻은 관측값도 기하학적으로 비슷하게 나타날 수 있어 이러한 판단이 어렵다. 따라서 신뢰할 수 있는 중첩 검출을 위해서는 로봇 간 대응 관계를 유효한 지도 관계로 승인하기 전에 충분한 공간적 문맥(Spatial Context)과 일관성 검증(Consistency Checking)이 필요하다.

로봇 간 루프 폐쇄(Cross-robot Loop Closure)는 기존의 단일 궤적 루프 폐쇄를 여러 로봇으로 확장한다. 로봇 A가 이전에 로봇 B가 지도화했던 위치를 관측하면 지도작성 시스템은 두 로봇의 궤적 사이에 상대 포즈 제약조건(Relative Pose Constraint)을 설정할 수 있다. 이 제약조건은 이전까지 독립적이었던 포즈 그래프(Pose Graph)를 연결하고, 최적화 시스템이 각 로봇과 지역 지도 사이의 공통 공간 관계를 추정할 수 있도록 한다.

잘못된 로봇 간 루프 폐쇄는 공유 지도(Shared Map)를 심각하게 왜곡할 수 있다. 창고에는 반복되는 선반, 동일한 형태의 기둥, 균일한 복도, 대칭적인 배치가 빈번하여 잘못된 장소 인식(False Place Recognition)이 발생할 수 있다. 따라서 후보 대응 관계가 전역 최적화에 영향을 주기 전에 기하학적 정합(Geometric Registration), 주변 관측과의 일관성, 변환의 타당성(Transformation Plausibility), 기타 센서 또는 의미론적 정보(Semantic Information)를 이용하여 검증해야 한다.

유효한 로봇 간 제약조건이 확보되면 포즈 그래프 최적화(Pose-graph Optimization)를 이용하여 여러 로봇의 궤적을 공동으로 정밀화할 수 있다. 오도메트리(Odometry), 지역 스캔 정합(Local Scan Matching), 개별 루프 폐쇄, 로봇 간 제약조건을 하나의 공통 최적화 문제로 결합한다. 이를 통해 단순히 불확실한 초기 추정값으로 포인트 클라우드를 겹치는 것이 아니라 누적 드리프트를 감소시키고 지역 지도들을 전역적으로 일관된 창고 공간 표현으로 정렬할 수 있다.

공유 지도는 일반적인 실내 AMR을 위한 2D 점유 격자(2D Occupancy Grid)로 표현하거나 수직 구조가 중요한 경우 3D 포인트 클라우드, 복셀(Voxel), 또는 하이브리드 표현(Hybrid Representation)으로 구성할 수 있다. 높은 랙, 메자닌(Mezzanine), 경사로, 자동 저장 시스템(Automated Storage System), 모바일 매니퓰레이터(Mobile Manipulator)가 존재하는 창고에서는 내비게이션 중심의 2D 계층과 더욱 풍부한 3D 구조 지도를 함께 유지하는 것이 유용할 수 있다.

지도 병합(Map Merging)에서는 기하학적 정렬(Geometric Alignment)과 지도 콘텐츠 융합(Map-content Fusion)을 구분해야 한다. 두 지도가 정확하게 정렬되어 있더라도 서로 다른 시점에서 관측되었다면 점유 정보가 충돌할 수 있다. 팔레트, 카트, 지게차, 임시 장벽, 재고는 관측 사이에 이동할 수 있기 때문이다. 따라서 융합 과정에서는 로봇 지도 사이의 모든 차이를 지도작성 오류의 증거로 처리해서는 안 된다.

영구적인 인프라(Persistent Infrastructure)는 일시적인 창고 객체보다 높은 중요도를 가져야 한다. 벽, 구조 기둥, 고정된 랙 프레임, 문, 충전 인프라, 영구 설치 기계는 위치추정을 위한 안정적인 기준을 제공한다. 반면 팔레트, 포장물, 작업자가 운전하는 차량, 임시 적치 영역은 필요한 경우 동적 계층(Dynamic Layer) 또는 단기 계층(Short-term Layer)을 통해 표현하는 것이 적절하다.

통신 제약조건(Communication Constraint)은 협업 지도작성 아키텍처에 큰 영향을 준다. 모든 로봇의 원시 라이다 스캔(Raw LiDAR Scan)을 지속적으로 전송하면 특히 대규모 플릿에서 상당한 통신 대역폭을 사용할 수 있다. 대신 로봇은 압축된 서브맵(Compressed Submap), 선택된 키프레임(Keyframe), 기술자(Descriptor), 포즈 그래프 제약조건 또는 지도 변경 정보만 교환할 수 있다. 이를 통해 협업 정렬과 최적화에 필요한 정보를 유지하면서 통신 요구량을 줄일 수 있다.

중앙집중형 아키텍처(Centralized Architecture)는 로봇의 관측 정보를 지도 서버(Map Server)에 수집하고 하나의 인프라 구성요소에서 로봇 간 정합, 전역 최적화(Global Optimization), 지도 배포(Map Distribution)를 수행할 수 있다. 이는 전역 일관성 관리를 단순화하지만 네트워크 연결성과 서버 가용성(Server Availability)에 대한 의존성을 발생시킨다. 따라서 일시적인 통신 단절에서도 로봇이 안전하게 내비게이션을 계속할 수 있도록 충분한 지역 자율성(Local Autonomy)을 유지해야 한다.

분산형 또는 탈중앙화 아키텍처(Distributed or Decentralized Architecture)는 로봇들이 직접 또는 여러 컴퓨팅 노드를 통해 지도 정보를 교환할 수 있도록 한다. 이러한 구조는 복원력(Resilience)을 높이고 단일 서버에 대한 의존성을 줄일 수 있지만 동기화, 일관성, 대역폭 할당, 분산 최적화(Distributed Optimization)가 더욱 어려워진다. 따라서 독립적인 지역 SLAM과 인프라 지원 전역 지도 최적화를 결합하는 하이브리드 아키텍처(Hybrid Architecture)가 실용적인 접근법이 될 수 있다.

창고 지도작성은 서로 다른 로봇에 상호 보완적인 탐색 영역을 할당하여 가속할 수 있다. 하나의 AMR은 입고 도크(Receiving Dock)를 지도화하고 다른 로봇들은 저장 통로, 생산 인터페이스 또는 출하 구역을 담당할 수 있다. 탐색 계획(Exploration Planning)은 불필요한 중복을 최소화하면서도 지도 병합에 필요한 신뢰할 수 있는 로봇 간 제약조건을 생성할 수 있도록 영역 사이에 충분한 중첩을 의도적으로 유지해야 한다.

협업 지도는 운영 플릿 자산(Operational Fleet Asset)이 된 이후 버전 관리(Version Control)를 적용해야 한다. 여러 로봇으로부터 생성된 후보 지도(Candidate Map)는 배포 전에 기하학적 검증, 경로 시험, 도킹 검증(Docking Verification), 의미론적 검토(Semantic Review)를 거칠 수 있다. 잘못된 지도 업데이트가 창고의 모든 로봇에 즉시 영향을 미치지 않도록 승인된 지도 버전은 롤백(Rollback)이 가능한 상태로 유지해야 한다.

지도 업데이트에는 충돌 해결 정책(Conflict-resolution Policy)이 필요하다. 한 로봇이 특정 통로가 차단된 것으로 관측했지만 다른 로봇이 최근 같은 위치를 자유 공간으로 관측했다면, 시스템은 이 차이가 일시적인 장애물인지 구조적인 변화인지 판단해야 한다. 관측 시각, 변화의 지속성(Persistence), 여러 로봇 간 일치 여부, 의미론적 분류(Semantic Classification), 신뢰도(Confidence)를 이용하여 해당 정보를 영구 지도에 반영할지 동적 계층에 유지할지 결정할 수 있다.

의미론적 정보는 협업 지도를 창고 운영에 더욱 유용하게 만들 수 있다. 저장 구역(Storage Zone), 상하역 도크(Loading Dock), 충전소(Charging Station), 엘리베이터, 제한 구역(Restricted Area), 교차로, 일방통행 통로(One-way Aisle), 도킹 지점(Docking Point), 생산 셀(Production Cell)을 기하학적 좌표와 연결할 수 있다. 이를 통해 로봇은 단순한 점유 공간 표현뿐만 아니라 창고 공간에 대한 공통의 운영 의미(Common Operational Interpretation)를 공유할 수 있다.

초기 지도작성이 완료된 이후에도 협업 관측(Collaborative Observation)은 위치추정 성능을 향상시킬 수 있다. 여러 로봇이 동일한 통로에서 반복적으로 낮은 스캔 정합 품질을 경험한다면 플릿은 해당 위치를 취약한 위치추정 영역(Weak Localization Region)으로 식별할 수 있다. 더 풍부한 센서를 탑재한 다른 로봇이 추가 관측을 수행하거나 인프라 랜드마크를 추가하여 해당 영역의 공유 위치 기준을 개선할 수 있다.

운영 모니터링(Operational Monitoring)에서는 지도 정렬 잔차(Map Alignment Residual), 로봇 간 루프 폐쇄, 최적화 일관성(Optimization Consistency), 지도 충돌(Map Conflict), 통신 지연시간, 위치추정 품질, 지도 버전, 데이터 최신성(Data Freshness)을 추적해야 한다. 플릿 수준 시각화(Fleet-level Visualization)를 통해 개별 로봇이 공통 지도에서 점진적으로 벗어나고 있는지 또는 특정 영역이 여러 로봇에서 반복적인 불일치를 발생시키는지를 확인할 수 있다.

하나의 오작동 로봇이 전체 플릿 지도를 손상시켜서는 안 되므로 장애 격리(Failure Isolation)가 필수적이다. 잘못된 보정, 손상된 라이다, 휠 슬립으로 인한 오도메트리 오차, 잘못된 타임스탬프는 체계적으로 왜곡된 지역 지도를 생성할 수 있다. 따라서 각 로봇이 제공하는 정보에는 품질 정보(Quality Information)를 함께 포함하여 의심스러운 데이터를 전역 지도에 통합하기 전에 거부하거나 격리(Quarantine)할 수 있어야 한다.

복구 메커니즘(Recovery Mechanism)은 로봇 수준 장애와 지도 수준 장애를 모두 지원해야 한다. 위치추정을 상실한 로봇은 공유 지도를 이용하여 재위치추정(Relocalization)을 수행할 수 있으며, 손상된 전역 지도 업데이트는 이전에 검증된 지도 버전으로 롤백할 수 있다. 통신이 단절되면 로봇은 지역 내비게이션을 계속 수행하고 연결이 복구된 이후 누적된 지도작성 정보를 다시 동기화할 수 있다.

검증(Validation)은 각 로봇을 독립적으로 평가하는 것이 아니라 실제 플릿 규모의 창고 운영을 재현해야 한다. 동시 지도작성, 중첩 및 비중첩 궤적, 반복적인 랙 통로, 통신 단절, 로봇 재시작, 임시 장애물, 구조적 변화, 잘못된 루프 폐쇄 후보, 지도 서버 중단, 장시간 오프라인 상태 이후 복귀한 로봇의 재통합 등을 시험에 포함해야 한다.

플릿 규모가 증가하면 협업 지도작성은 SLAM 문제인 동시에 데이터 관리(Data Management) 문제가 된다. 지도 관측값, 키프레임, 로봇 궤적, 의미론적 주석(Semantic Annotation), 타임스탬프, 신뢰도 값, 지도 버전의 추적 가능성(Traceability)을 유지해야 한다. 수백 대의 로봇이 현재 운용 영역과 관련 없는 정보를 지속적으로 교환하지 않도록 효율적인 저장 및 선택적 배포(Selective Distribution)가 필요하다.

성숙한 창고 협업 지도작성 아키텍처(Warehouse Collaborative Mapping Architecture)는 독립적인 지역 SLAM, 로봇 간 장소 인식(Cross-robot Place Recognition), 지도 병합, 포즈 그래프 최적화, 통신 관리, 충돌 해결, 통제된 지도 배포를 하나의 체계로 결합한다. 목표는 단순히 여러 로봇의 지도를 합치는 것이 아니라 위치추정, 내비게이션, 플릿 조정(Fleet Coordination), 장기적인 창고 운영을 향상시키는 신뢰할 수 있는 공유 공간 기준(Shared Spatial Reference)을 지속적으로 유지하는 것이다.

##  

## 12.08. Long Term Map Management 2 Year Operation Case

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

Long-term map management becomes a core operational function when autonomous robots remain deployed in the same facility for months or years. A map created during commissioning cannot be assumed to remain correct throughout a two-year operation. Equipment relocation, construction, new racks, changed traffic rules, seasonal effects, and infrastructure modifications gradually separate the operational environment from the original SLAM reference.

A production map should therefore be treated as a controlled system asset rather than a static SLAM output file. Each released map requires an identifiable version, creation date, validation state, compatible robot configuration, coordinate-frame definition, and change history. This allows localization problems to be traced to the exact spatial reference used by a robot at a particular point in its operational history.

The architecture should separate relatively permanent geometry from information expected to change frequently. Walls, columns, structural frames, fixed machines, and permanent infrastructure belong to a stable map layer. Pallets, carts, temporary barriers, parked vehicles, movable equipment, and short-term storage conditions should normally remain in dynamic or operational layers rather than continuously modifying the localization reference.

During normal missions, robots continuously generate observations that can be compared with the approved map. Persistent disagreement between current sensor measurements and expected geometry can indicate environmental change. However, a single mismatch should not automatically trigger a map update because temporary obstacles, people, sensor noise, localization error, or short-lived operational conditions can produce similar differences.

Change detection therefore requires temporal persistence and confidence. If many observations collected over hours, days, or repeated missions consistently show that a structure has appeared, disappeared, or moved, the system can classify the region as a candidate map change. Observations from multiple robots provide additional evidence and reduce the probability that one faulty sensor or poorly localized robot creates an incorrect update.

A useful two-year architecture maintains an approved production map separately from candidate changes. Robots continue localizing against the validated reference while newly detected differences accumulate within a staging layer. Engineers or automated validation processes can review these candidate modifications before they are promoted into the next production map version, preventing uncontrolled environmental observations from immediately affecting fleet navigation.

Map updates can be classified according to operational impact. Minor geometric changes may have little influence on localization or route planning, while relocated walls, modified rack layouts, new doors, changed docking stations, or reconstructed intersections can directly affect autonomous operation. Update priority should therefore consider not only geometric difference but also its relationship to localization, navigation, safety, and mission execution.

Semantic information requires the same lifecycle discipline as geometric data. Charging stations, docking points, restricted zones, one-way aisles, elevators, inspection locations, loading areas, and mission landmarks can change independently from physical geometry. A wall may remain unchanged while the traffic policy around it changes, meaning semantic and operational map layers need explicit versioning rather than being embedded invisibly inside geometry.

Long-term operation also reveals localization weak points that may not appear during initial commissioning. Repeated scan-matching failures, high pose covariance, frequent relocalization, or inconsistent loop closures in the same region indicate that the map may provide insufficient constraints there. These statistics can guide targeted remapping, additional landmarks, sensor improvements, or modification of the localization strategy for specific areas.

Environmental appearance can vary even when permanent infrastructure remains unchanged. Warehouses may experience changing inventory density, outdoor robots encounter vegetation and weather variation, and industrial sites can accumulate temporary structures. A robust long-term map should preserve features that remain useful across these variations instead of overfitting the localization reference to the exact environmental appearance observed during one mapping session.

Map aging can therefore be measured using operational evidence rather than calendar time alone. A two-year-old region that still produces stable localization may remain valid, while a map created only weeks earlier may already be obsolete after facility reconstruction. Map health indicators can combine observation mismatch, localization residuals, robot agreement, environmental-change persistence, and mission failure statistics to identify areas requiring attention.

Periodic remapping remains useful even when continuous change detection is available. Scheduled surveys can provide systematic coverage of regions that normal robot missions rarely visit. The frequency does not need to be identical across the facility: high-change logistics areas may require more frequent inspection, while structurally stable corridors or fixed production zones may remain valid for substantially longer periods.

When remapping is required, rebuilding the entire facility map is not always necessary. Local submaps can be regenerated for changed regions and aligned with unchanged portions of the validated global reference. This reduces validation effort and preserves stable coordinate relationships used by docking stations, mission databases, infrastructure interfaces, and other systems that depend on persistent map coordinates.

Coordinate-frame stability is especially important during long-term updates. Arbitrarily rebuilding a global map can shift coordinates even when the physical facility has barely changed. Such shifts can invalidate stored waypoints, semantic annotations, inspection locations, and external-system references. Map-update procedures should therefore preserve the established global frame whenever possible and explicitly manage transformations when frame changes are unavoidable.

Before release, a candidate map version should pass geometric and operational validation. Alignment quality, localization repeatability, navigation routes, docking accuracy, restricted regions, mission landmarks, and critical intersections should be checked. Fleet deployment should occur only after the candidate demonstrates that it maintains or improves required performance relative to the currently approved map.

Map deployment should be controlled in the same manner as a software release. A new version can first be distributed to a limited validation robot or small fleet subset, allowing real missions to expose unexpected problems before full deployment. If localization quality, navigation success, or operational metrics deteriorate, deployment can be stopped without immediately affecting every robot in the facility.

Rollback capability is essential for long-term map governance. The fleet should retain previously validated map versions together with their associated metadata and configuration dependencies. If a newly released map causes localization instability, route errors, or docking problems, robots can return to a known-good reference while the candidate version is investigated and corrected.

Compatibility must also be managed as robot hardware evolves during two years of operation. A fleet may receive new LiDAR models, cameras, computing platforms, firmware, calibration parameters, or localization algorithms. A map that works well with one sensing configuration may behave differently with another. Map versions should therefore record relevant sensor and localization dependencies instead of assuming that all fleet generations interpret the environment identically.

Multi-robot fleets provide valuable information for map-health assessment. If one robot reports a discrepancy while twenty others localize normally, the problem may belong to that robot rather than the environment. If many independent robots report consistent residuals at the same location, a real environmental change becomes more likely. Fleet-level statistics therefore provide an important mechanism for separating robot faults from map faults.

Failure isolation prevents corrupted observations from entering the map lifecycle. Robots with calibration problems, damaged sensors, abnormal timing, wheel-slip errors, or unstable localization should have their mapping contributions reduced, rejected, or quarantined. Data provenance should identify which robot, sensor configuration, software version, mission, and timestamp produced each significant candidate map change.

Long-term map storage should preserve more than the latest occupancy grid or point cloud. Production versions, candidate maps, submaps, keyframes, change records, semantic annotations, validation results, deployment history, and rollback references form an operational record. Retention policies can remove unnecessary raw data while preserving enough evidence to reproduce important map decisions and diagnose historical failures.

Map distribution must ensure that robots know exactly which version they are using. Version identifiers, integrity checks, compatibility metadata, and deployment status should accompany the map package. Interrupted transfers or partially updated map components must not leave a robot operating with inconsistent geometry and semantic layers. Update procedures should therefore support atomic activation or equivalent consistency protection.

Monitoring continues after deployment because successful installation does not guarantee successful operation. Localization residuals, relocalization frequency, route completion, docking accuracy, map mismatch, collision-map behavior, and operator interventions should be compared before and after each release. These observations provide evidence for accepting the new version permanently or initiating rollback and investigation.

Over a two-year lifecycle, map governance becomes closely connected to fleet operations and maintenance. Facility modifications should ideally enter the mapping workflow before robots encounter them unexpectedly. Planned construction, rack relocation, new machinery, changed safety zones, and revised traffic policies can generate scheduled map updates, while autonomous change detection remains responsible for identifying unplanned differences.

A mature long-term map management system therefore creates a closed operational loop: validated maps support localization, robot observations reveal environmental change, candidate updates are accumulated and verified, new versions are tested and deployed, and operational performance determines whether they remain active. This transforms SLAM mapping from a commissioning activity into a continuously governed infrastructure for dependable autonomous operation.

자율 로봇이 동일한 시설에서 수개월 또는 수년 동안 지속적으로 운용될 경우 장기 지도 관리(Long-term Map Management)는 핵심적인 운영 기능이 된다. 초기 구축(Commissioning) 과정에서 생성된 지도가 2년의 운용 기간 전체에 걸쳐 계속 정확하다고 가정할 수는 없다. 장비 재배치, 공사, 신규 랙, 교통 규칙 변경, 계절적 영향, 인프라 변경 등이 누적되면서 실제 운영 환경과 초기 SLAM 기준 지도(SLAM Reference) 사이의 차이가 점차 증가한다.

따라서 생산용 지도(Production Map)는 정적인 SLAM 출력 파일이 아니라 통제된 시스템 자산(Controlled System Asset)으로 관리해야 한다. 배포되는 각 지도에는 식별 가능한 버전(Version), 생성 날짜, 검증 상태(Validation State), 호환 가능한 로봇 구성, 좌표 프레임 정의(Coordinate-frame Definition), 변경 이력(Change History)이 필요하다. 이를 통해 특정 시점에 로봇이 사용했던 정확한 공간 기준(Spatial Reference)까지 추적하여 위치추정 문제를 분석할 수 있다.

아키텍처에서는 상대적으로 영구적인 기하학 구조와 자주 변경될 것으로 예상되는 정보를 분리해야 한다. 벽, 기둥, 구조 프레임, 고정 기계, 영구 인프라는 안정 지도 계층(Stable Map Layer)에 포함한다. 팔레트, 카트, 임시 장벽, 주차 차량, 이동 가능한 장비, 단기 적치 상태는 위치추정 기준 지도를 지속적으로 변경하기보다 일반적으로 동적 계층(Dynamic Layer) 또는 운영 계층(Operational Layer)에 유지해야 한다.

정상적인 임무 수행 과정에서 로봇은 승인된 지도와 비교할 수 있는 관측값을 지속적으로 생성한다. 현재 센서 측정값과 예상되는 기하학 구조 사이의 지속적인 불일치는 환경 변화(Environmental Change)를 나타낼 수 있다. 그러나 일시적인 장애물, 사람, 센서 노이즈, 위치추정 오차 또는 단기적인 운영 조건도 유사한 차이를 발생시킬 수 있으므로 단 한 번의 불일치만으로 지도를 자동 갱신해서는 안 된다.

따라서 변화 검출(Change Detection)에는 시간적 지속성(Temporal Persistence)과 신뢰도(Confidence)가 필요하다. 수시간, 수일 또는 반복적인 임무에서 수집된 여러 관측값이 특정 구조물이 새롭게 나타났거나 사라졌거나 이동했음을 일관되게 보여주는 경우 해당 영역을 후보 지도 변경(Candidate Map Change)으로 분류할 수 있다. 여러 로봇의 관측값을 함께 사용하면 추가적인 증거를 확보하여 하나의 고장 센서나 위치추정이 불안정한 로봇이 잘못된 업데이트를 생성할 가능성을 줄일 수 있다.

실용적인 2년 운용 아키텍처에서는 승인된 생산 지도(Approved Production Map)와 후보 변경 사항을 분리하여 관리한다. 로봇은 검증된 기준 지도에 대해 계속 위치추정을 수행하고, 새롭게 검출된 차이는 스테이징 계층(Staging Layer)에 누적한다. 엔지니어나 자동 검증 프로세스가 이러한 후보 변경을 검토한 후 다음 생산 지도 버전으로 승격하도록 하여 통제되지 않은 환경 관측값이 즉시 플릿 내비게이션(Fleet Navigation)에 영향을 주는 것을 방지할 수 있다.

지도 업데이트(Map Update)는 운영 영향도에 따라 분류할 수 있다. 작은 기하학적 변화는 위치추정이나 경로 계획에 거의 영향을 주지 않을 수 있지만, 이동된 벽, 변경된 랙 배치, 새로운 문, 변경된 도킹 스테이션(Docking Station), 재구성된 교차로는 자율 운용에 직접적인 영향을 줄 수 있다. 따라서 업데이트 우선순위는 단순한 기하학적 차이뿐만 아니라 위치추정, 내비게이션, 안전, 임무 수행과의 관계까지 고려해야 한다.

의미론적 정보(Semantic Information)에도 기하학 데이터와 동일한 수명주기 관리(Lifecycle Discipline)가 필요하다. 충전소, 도킹 지점, 제한 구역, 일방통행 통로, 엘리베이터, 점검 위치, 적재 구역, 임무 랜드마크(Mission Landmark)는 물리적 기하학 구조와 독립적으로 변경될 수 있다. 벽 자체는 변하지 않아도 주변의 교통 정책이 변경될 수 있으므로 의미론적 계층과 운영 지도 계층에도 기하학 정보 내부에 암묵적으로 포함시키는 대신 명시적인 버전 관리가 필요하다.

장기 운용에서는 초기 구축 과정에서 발견되지 않았던 위치추정 취약 지점(Localization Weak Point)이 나타날 수 있다. 동일한 영역에서 반복적인 스캔 정합 실패(Scan-matching Failure), 높은 포즈 공분산(Pose Covariance), 빈번한 재위치추정(Relocalization), 일관되지 않은 루프 폐쇄(Loop Closure)가 발생한다면 해당 지역의 지도가 충분한 제약조건을 제공하지 못하고 있음을 의미할 수 있다. 이러한 통계는 선택적 재지도작성, 추가 랜드마크, 센서 개선 또는 특정 영역의 위치추정 전략 변경에 활용할 수 있다.

영구적인 인프라가 변경되지 않더라도 환경의 외형(Environmental Appearance)은 달라질 수 있다. 창고에서는 재고 밀도가 변화하고, 실외 로봇은 식생과 날씨 변화의 영향을 받으며, 산업 현장에서는 임시 구조물이 지속적으로 추가될 수 있다. 강건한 장기 지도는 한 번의 지도작성 과정에서 관측된 특정 환경 상태에 지나치게 적합화(Overfitting)하기보다 이러한 변화 속에서도 계속 위치추정에 활용할 수 있는 특징을 보존해야 한다.

따라서 지도 노후화(Map Aging)는 단순한 경과 시간이 아니라 운영 증거(Operational Evidence)를 기반으로 평가할 수 있다. 2년 전에 생성된 지도 영역이라도 안정적인 위치추정을 계속 제공한다면 여전히 유효할 수 있으며, 불과 몇 주 전에 생성된 지도라도 시설 재구축 이후에는 이미 오래된 정보가 될 수 있다. 지도 상태 지표(Map Health Indicator)는 관측 불일치, 위치추정 잔차, 로봇 간 일치도, 환경 변화의 지속성, 임무 실패 통계를 결합하여 관리가 필요한 영역을 식별할 수 있다.

연속적인 변화 검출 기능이 존재하더라도 주기적인 재지도작성(Periodic Remapping)은 여전히 유용하다. 계획된 조사 작업(Scheduled Survey)은 정상적인 로봇 임무에서 거의 방문하지 않는 영역까지 체계적으로 확인할 수 있다. 모든 시설 영역에 동일한 주기를 적용할 필요는 없으며, 변화가 많은 물류 영역은 더 자주 점검하고 구조적으로 안정적인 복도나 고정 생산 구역은 훨씬 긴 기간 동안 기존 지도를 유지할 수 있다.

재지도작성이 필요하더라도 시설 전체 지도를 항상 새로 구축할 필요는 없다. 변경된 영역에 대해서만 지역 서브맵(Local Submap)을 다시 생성하고 변경되지 않은 검증된 전역 기준 지도(Global Reference)에 정합할 수 있다. 이를 통해 검증 작업량을 줄이는 동시에 도킹 스테이션, 임무 데이터베이스, 인프라 인터페이스 및 지속적인 지도 좌표에 의존하는 다른 시스템에서 사용하는 안정적인 좌표 관계를 유지할 수 있다.

장기 지도 업데이트에서는 좌표 프레임 안정성(Coordinate-frame Stability)이 특히 중요하다. 전역 지도를 임의로 다시 구축하면 실제 시설이 거의 변하지 않았음에도 좌표가 이동할 수 있다. 이러한 좌표 변화는 저장된 웨이포인트(Waypoint), 의미론적 주석(Semantic Annotation), 점검 위치, 외부 시스템 기준을 무효화할 수 있다. 따라서 지도 업데이트 절차에서는 가능한 한 기존 전역 프레임을 유지하고 프레임 변경이 불가피한 경우에는 변환 관계를 명시적으로 관리해야 한다.

후보 지도 버전(Candidate Map Version)은 배포 전에 기하학적 검증과 운영 검증을 통과해야 한다. 정렬 품질(Alignment Quality), 위치추정 반복성(Localization Repeatability), 내비게이션 경로, 도킹 정확도, 제한 구역, 임무 랜드마크, 주요 교차로 등을 확인해야 한다. 후보 지도가 현재 승인된 지도와 비교하여 요구되는 성능을 유지하거나 개선한다는 것이 확인된 이후에만 플릿에 배포해야 한다.

지도 배포(Map Deployment)는 소프트웨어 릴리스(Software Release)와 유사한 방식으로 통제해야 한다. 새로운 버전을 먼저 제한된 검증 로봇이나 소규모 플릿에 배포하여 전체 배포 전에 실제 임무를 통해 예상하지 못한 문제를 확인할 수 있다. 위치추정 품질, 내비게이션 성공률 또는 운영 지표가 악화되면 시설 내 모든 로봇에 즉시 영향을 주지 않고 배포를 중단할 수 있다.

롤백 기능(Rollback Capability)은 장기 지도 거버넌스(Map Governance)의 필수 요소이다. 플릿은 이전에 검증된 지도 버전과 관련 메타데이터(Metadata), 구성 의존성(Configuration Dependency)을 함께 보존해야 한다. 새롭게 배포된 지도가 위치추정 불안정, 경로 오류 또는 도킹 문제를 발생시키는 경우 후보 버전을 조사하고 수정하는 동안 로봇을 기존의 정상 동작이 확인된 기준 지도(Known-good Reference)로 복귀시킬 수 있어야 한다.

2년의 운용 기간 동안 로봇 하드웨어가 발전할 수 있으므로 호환성(Compatibility)도 관리해야 한다. 플릿에 새로운 라이다 모델, 카메라, 컴퓨팅 플랫폼, 펌웨어, 보정 파라미터 또는 위치추정 알고리즘이 적용될 수 있다. 특정 센서 구성에서 잘 동작했던 지도가 다른 구성에서는 다르게 동작할 수 있다. 따라서 모든 플릿 세대가 동일하게 환경을 해석한다고 가정하지 말고 지도 버전에 관련 센서 및 위치추정 의존성을 기록해야 한다.

다중 로봇 플릿(Multi-robot Fleet)은 지도 상태 평가(Map-health Assessment)에 유용한 정보를 제공한다. 한 대의 로봇만 불일치를 보고하고 다른 20대의 로봇이 정상적으로 위치추정을 수행한다면 환경보다 해당 로봇 자체에 문제가 있을 가능성이 있다. 반대로 여러 독립적인 로봇이 동일한 위치에서 일관된 잔차를 보고한다면 실제 환경 변화일 가능성이 높아진다. 따라서 플릿 수준 통계(Fleet-level Statistics)는 로봇 장애와 지도 장애를 구분하는 중요한 수단이 된다.

장애 격리(Failure Isolation)는 손상된 관측값이 지도 수명주기(Map Lifecycle)에 유입되는 것을 방지한다. 보정 문제, 손상된 센서, 비정상적인 타이밍, 휠 슬립 오차 또는 불안정한 위치추정을 가진 로봇의 지도작성 데이터는 영향도를 낮추거나 거부하거나 격리(Quarantine)해야 한다. 데이터 출처 추적(Data Provenance)을 통해 중요한 후보 지도 변경이 어떤 로봇, 센서 구성, 소프트웨어 버전, 임무 및 타임스탬프에서 생성되었는지 확인할 수 있어야 한다.

장기 지도 저장(Long-term Map Storage)은 최신 점유 격자(Occupancy Grid) 또는 포인트 클라우드만 보존해서는 안 된다. 생산 지도 버전, 후보 지도, 서브맵, 키프레임(Keyframe), 변경 기록, 의미론적 주석, 검증 결과, 배포 이력, 롤백 기준은 하나의 운영 기록(Operational Record)을 구성한다. 보존 정책(Retention Policy)을 통해 불필요한 원시 데이터는 제거하면서도 중요한 지도 관련 의사결정을 재현하고 과거 장애를 분석할 수 있는 충분한 증거를 유지할 수 있다.

지도 배포 과정에서는 각 로봇이 정확히 어떤 버전을 사용하고 있는지 확인할 수 있어야 한다. 버전 식별자(Version Identifier), 무결성 검사(Integrity Check), 호환성 메타데이터, 배포 상태를 지도 패키지와 함께 관리해야 한다. 전송 중단이나 부분적으로 갱신된 지도 구성요소 때문에 로봇이 서로 일치하지 않는 기하학 계층과 의미론적 계층을 사용하는 상황이 발생해서는 안 된다. 따라서 업데이트 절차는 원자적 활성화(Atomic Activation) 또는 이에 상응하는 일관성 보호 기능을 지원해야 한다.

성공적으로 설치되었다고 해서 실제 운용에서도 성공적이라고 보장할 수 없으므로 배포 이후에도 모니터링(Monitoring)을 계속해야 한다. 위치추정 잔차, 재위치추정 빈도, 경로 완료율, 도킹 정확도, 지도 불일치, 충돌 지도 동작(Collision-map Behavior), 작업자 개입(Operator Intervention)을 각 릴리스 전후로 비교해야 한다. 이러한 관측 결과는 새로운 버전을 최종 승인할 것인지 또는 롤백 및 추가 조사를 수행할 것인지 판단하는 근거가 된다.

2년의 수명주기 동안 지도 거버넌스는 플릿 운영 및 유지보수와 긴밀하게 연결된다. 시설 변경 사항은 가능하면 로봇이 예기치 않게 이를 발견하기 전에 지도 관리 워크플로(Map Management Workflow)에 반영하는 것이 바람직하다. 계획된 공사, 랙 재배치, 신규 기계, 변경된 안전 구역, 수정된 교통 정책은 계획된 지도 업데이트(Scheduled Map Update)로 처리하고, 자율 변화 검출(Autonomous Change Detection)은 계획되지 않은 차이를 발견하는 역할을 담당할 수 있다.

성숙한 장기 지도 관리 시스템(Long-term Map Management System)은 폐루프 운영 구조(Closed Operational Loop)를 형성한다. 검증된 지도는 위치추정을 지원하고, 로봇의 관측값은 환경 변화를 발견하며, 후보 업데이트는 누적 및 검증되고, 새로운 버전은 시험을 거쳐 배포되며, 실제 운영 성능을 기반으로 해당 버전의 지속 사용 여부를 결정한다. 이를 통해 SLAM 지도작성은 초기 구축 단계의 일회성 작업에서 벗어나 신뢰할 수 있는 자율 운용을 지속적으로 지원하는 관리형 인프라(Managed Infrastructure)로 발전한다.

##  

## 12.09. SLAM Failure Analysis and Recovery Case

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

SLAM failure in a production robot rarely appears as a single instantaneous event. It often begins with gradual degradation in scan matching, visual tracking, inertial consistency, or map alignment before the estimated pose becomes unusable. Effective failure analysis therefore requires continuous observation of localization health so that abnormal behavior can be detected before it propagates into navigation, planning, or vehicle-control failures.

A useful failure model separates sensing, calibration, estimation, environment, computation, and map-related causes. LiDAR contamination, camera occlusion, IMU vibration, encoder errors, incorrect extrinsics, timestamp offsets, processor overload, corrupted maps, or environmental changes can produce similar localization symptoms. Diagnosis must therefore identify the underlying failure mechanism rather than reacting only to the final pose error.

Sensor degradation is one of the most common failure sources. Dust, rain, condensation, reflective surfaces, direct sunlight, damaged optics, loose connectors, or mechanical vibration can reduce measurement quality without completely stopping a sensor. A production SLAM system should monitor measurement rate, signal quality, missing data, abnormal noise, and sensor-specific diagnostics so that degraded observations can be distinguished from estimator instability.

Calibration errors can create persistent systematic failures that are difficult to recognize from individual measurements. Small errors in LiDAR-to-IMU, camera-to-body, or wheel-to-base transformations can generate distorted maps and biased trajectories. Mechanical impacts, maintenance work, sensor replacement, or mounting deformation can change previously valid calibration parameters, making periodic calibration verification important throughout the robot lifecycle.

Temporal synchronization failures can be equally damaging. A LiDAR scan, camera image, IMU measurement, and wheel encoder sample describe the robot at different moments unless their timestamps are correctly aligned. Network delay, clock drift, software buffering, or configuration errors can create time offsets that appear as geometric misalignment. The resulting failure often becomes more severe as robot speed or angular motion increases.

Environmental degeneracy occurs when the surroundings do not provide enough observable information to constrain motion. Long corridors, repetitive warehouse racks, featureless walls, open spaces, tunnels, glass surfaces, or uniform floors can weaken scan or visual registration. The estimator should recognize reduced observability and increase uncertainty rather than continuing to output a precise-looking pose that is poorly supported by measurements.

Dynamic environments introduce another failure mechanism. People, forklifts, vehicles, doors, machinery, vegetation, and moving inventory can dominate sensor observations and violate the assumption that mapped landmarks are stationary. Robust estimation, semantic filtering, temporal consistency, and dynamic-object rejection can reduce this influence, but the system must still detect situations where stable environmental information becomes insufficient for reliable localization.

Map inconsistency becomes increasingly important during long-term deployment. Construction, relocated equipment, changed racks, modified walls, or outdated semantic information can cause current observations to disagree with the reference map. The robot may initially interpret this disagreement as localization error even when its sensors are functioning correctly. Failure analysis should therefore distinguish an incorrect robot pose from an obsolete or locally corrupted map.

Estimator divergence can occur when erroneous measurements repeatedly enter the state solution. Incorrect loop closures, poor scan registration, underestimated sensor covariance, excessive inertial bias, or inconsistent constraints can gradually distort the trajectory. Monitoring innovation values, registration residuals, covariance, optimization cost, and constraint consistency helps identify divergence before the state estimate becomes catastrophically incorrect.

Loop-closure failure deserves special attention because a false closure can alter a large portion of the map or trajectory. Repetitive environments can cause two different places to appear geometrically or visually similar. Candidate closures should therefore pass geometric verification and consistency tests before being accepted. Large corrections produced by a newly introduced loop constraint should also trigger additional validation rather than being trusted automatically.

Computation can fail even when sensing and algorithms are correct. CPU or GPU saturation, memory exhaustion, thermal throttling, communication congestion, storage delays, or overloaded middleware can cause dropped frames and irregular processing latency. A SLAM system operating with stale sensor data may produce localization errors that resemble sensor failure, making resource utilization and end-to-end timing essential diagnostic signals.

Failure detection should combine multiple indicators instead of relying on one threshold. Scan-matching score, reprojection error, feature count, IMU residuals, wheel consistency, pose covariance, map overlap, loop-closure confidence, processing latency, and sensor health can collectively describe localization quality. A health manager can combine these signals into operational states such as normal, degraded, critical, or lost localization.

A degraded state should influence robot behavior before localization is completely lost. The robot may reduce speed, increase obstacle clearance, avoid narrow passages, suspend precision docking, or move toward a region containing stronger localization features. Such behavior limits the consequences of uncertainty and provides the SLAM system with additional time or improved observations from which to recover.

When confidence falls below a safe threshold, controlled stopping is often preferable to continuing autonomous motion. The robot should reach a stable configuration if possible, preserve recent sensor and estimator data, and prevent navigation commands from assuming that the last estimated pose remains valid indefinitely. Safety behavior should remain independent enough that a SLAM failure cannot disable the mechanisms responsible for preventing hazardous motion.

Relocalization is the primary recovery mechanism after pose tracking is lost. Current LiDAR scans or visual observations can be compared against previously stored maps, submaps, keyframes, or place descriptors. Multiple candidate poses may initially be generated, and geometric verification can determine which hypothesis is consistent with current measurements. Autonomous navigation should resume only after pose confidence becomes sufficiently strong.

Recovery can also exploit recent trajectory history. If localization fails after entering a difficult region, the robot may retain a short-term estimate of where reliable tracking was last available. Backtracking toward that region can restore stronger map overlap or environmental features. This strategy is particularly useful in repetitive corridors, temporary construction areas, vegetation, or locations where sensor visibility suddenly deteriorates.

Multi-sensor systems can continue operating through partial failures by changing sensor weighting or localization mode. A LiDAR failure may shift greater responsibility toward visual-inertial estimation, while poor lighting may increase reliance on LiDAR and IMU. Wheel odometry, GNSS, radar, depth sensing, or infrastructure landmarks can provide additional constraints, but transitions must avoid abrupt coordinate jumps or unjustified confidence.

A recovery manager should distinguish temporary degradation from persistent hardware or map faults. Repeated relocalization failure at one location may indicate a changed environment, while failures that follow one robot across many locations may indicate sensor or calibration problems. Fleet-level comparison is valuable because observations from other robots can determine whether the failure is location-specific or platform-specific.

Fault isolation prevents one failing component from corrupting the entire estimation chain. Measurements with abnormal timestamps, implausible motion, excessive residuals, or inconsistent covariance can be rejected or down-weighted. If a sensor is identified as unreliable, the estimator can temporarily exclude it while preserving diagnostic information for maintenance and later root-cause analysis.

Post-failure analysis requires synchronized logs rather than isolated error messages. Sensor data, estimated poses, transforms, covariance, optimization residuals, loop closures, CPU and GPU load, network latency, map version, software configuration, and navigation commands should share a consistent timeline. This allows engineers to reconstruct what happened before, during, and after localization degradation instead of examining disconnected subsystem records.

Root-cause analysis should separate the initiating event from secondary consequences. For example, vibration may corrupt IMU measurements, causing scan deskewing errors, which then reduce LiDAR registration quality and finally produce navigation oscillation. Treating navigation oscillation as the original failure would lead to an ineffective corrective action. A causal timeline is therefore essential for production debugging.

Recovery performance should be measured explicitly. Useful metrics include time to detect degradation, time to safe state, relocalization time, recovery success rate, distance traveled under degraded localization, pose error after recovery, and number of operator interventions. These measurements allow different recovery strategies to be compared and reveal whether software updates actually improve system resilience.

Validation should deliberately inject realistic failure conditions. Tests can include sensor disconnection, delayed timestamps, degraded LiDAR returns, camera occlusion, IMU vibration, wheel slip, false loop-closure candidates, outdated maps, processor overload, communication loss, and abrupt environmental changes. The objective is not merely to demonstrate nominal SLAM accuracy but to verify predictable behavior when assumptions fail.

Fleet operation provides an additional recovery resource. A robot that cannot localize may request a validated map region, recent environmental observations, or infrastructure information obtained by another robot. Fleet monitoring can also detect repeated failures at the same location and classify that area as a localization risk zone until mapping or infrastructure improvements are completed.

A mature SLAM failure-management architecture therefore forms a closed loop of health monitoring, degradation detection, safe-state transition, fault isolation, relocalization, recovery validation, and root-cause analysis. Reliable autonomy does not require SLAM to never fail; it requires failures to become observable, bounded, diagnosable, and recoverable without allowing localization uncertainty to propagate uncontrollably into robot behavior.

생산 환경에서 로봇의 SLAM 장애(SLAM Failure)는 하나의 순간적인 사건으로 발생하기보다 점진적인 성능 저하로 시작되는 경우가 많다. 스캔 정합(Scan Matching), 시각 추적(Visual Tracking), 관성 데이터 일관성(Inertial Consistency), 지도 정렬(Map Alignment)의 성능이 점차 악화된 이후 최종적으로 추정 포즈(Pose)를 사용할 수 없는 상태에 도달할 수 있다. 따라서 효과적인 장애 분석을 위해서는 이상 상태가 내비게이션, 계획 또는 차량 제어 장애로 확대되기 전에 위치추정 상태(Localization Health)를 지속적으로 관찰해야 한다.

유용한 장애 모델(Failure Model)은 원인을 센싱(Sensing), 보정(Calibration), 상태 추정(Estimation), 환경(Environment), 연산(Computation), 지도(Map) 관련 문제로 구분한다. 라이다 오염, 카메라 가림, IMU 진동, 엔코더 오류, 잘못된 외부 파라미터(Extrinsics), 타임스탬프 오프셋(Timestamp Offset), 프로세서 과부하, 손상된 지도, 환경 변화 등은 서로 유사한 위치추정 이상 증상을 발생시킬 수 있다. 따라서 최종적인 포즈 오차에만 대응하지 말고 근본적인 장애 메커니즘을 식별해야 한다.

센서 성능 저하(Sensor Degradation)는 가장 일반적인 장애 원인 중 하나이다. 먼지, 비, 결로, 반사 표면, 직사광선, 손상된 광학계, 느슨한 커넥터 또는 기계적 진동은 센서를 완전히 정지시키지 않으면서도 측정 품질을 저하시킬 수 있다. 생산용 SLAM 시스템은 측정 주기, 신호 품질, 누락 데이터, 비정상적인 노이즈, 센서별 진단 정보를 모니터링하여 성능이 저하된 관측값과 상태 추정기 자체의 불안정을 구분할 수 있어야 한다.

보정 오차(Calibration Error)는 개별 측정값만으로는 식별하기 어려운 지속적이고 체계적인 장애를 발생시킬 수 있다. 라이다-IMU, 카메라-몸체(Camera-to-body), 휠-베이스(Wheel-to-base) 사이의 작은 변환 오차도 왜곡된 지도와 편향된 궤적을 생성할 수 있다. 기계적 충격, 유지보수 작업, 센서 교체 또는 장착부 변형으로 기존의 유효한 보정 파라미터가 변경될 수 있으므로 로봇 수명주기 전체에서 주기적인 보정 검증(Calibration Verification)이 중요하다.

시간 동기화 장애(Temporal Synchronization Failure)도 동일하게 심각한 문제를 발생시킬 수 있다. 라이다 스캔, 카메라 영상, IMU 측정값, 휠 엔코더 샘플은 타임스탬프가 정확하게 정렬되지 않으면 서로 다른 시점의 로봇 상태를 나타낸다. 네트워크 지연, 클록 드리프트(Clock Drift), 소프트웨어 버퍼링 또는 설정 오류는 기하학적 정렬 오류처럼 보이는 시간 오프셋을 만들 수 있다. 이러한 장애는 로봇 속도나 각운동이 증가할수록 더욱 심각하게 나타나는 경우가 많다.

환경적 퇴화(Environmental Degeneracy)는 주변 환경이 로봇의 움직임을 제한하기 위한 충분한 관측 정보를 제공하지 못할 때 발생한다. 긴 복도, 반복적인 창고 랙, 특징이 부족한 벽, 개방 공간, 터널, 유리 표면, 균일한 바닥 등은 스캔 또는 시각 정합을 약화시킬 수 있다. 상태 추정기는 측정값의 지원이 부족한 포즈를 정밀한 값처럼 계속 출력하기보다 관측 가능성(Observability)의 감소를 인식하고 불확실성을 증가시켜야 한다.

동적 환경(Dynamic Environment)은 또 다른 장애 메커니즘을 발생시킨다. 사람, 지게차, 차량, 문, 기계, 식생, 이동하는 재고 등이 센서 관측의 상당 부분을 차지하면 지도상의 랜드마크가 정적이라는 가정이 성립하지 않을 수 있다. 강건한 상태 추정(Robust Estimation), 의미론적 필터링(Semantic Filtering), 시간적 일관성(Temporal Consistency), 동적 객체 제거(Dynamic-object Rejection)를 통해 이러한 영향을 줄일 수 있지만 안정적인 환경 정보 자체가 부족해지는 상황도 검출해야 한다.

장기 운용에서는 지도 불일치(Map Inconsistency)가 점점 중요한 장애 원인이 된다. 공사, 장비 재배치, 랙 변경, 벽 구조 변경 또는 오래된 의미론적 정보로 인해 현재 관측값이 기준 지도(Reference Map)와 일치하지 않을 수 있다. 이 경우 센서가 정상적으로 동작하고 있어도 로봇은 처음에는 이러한 차이를 위치추정 오류로 해석할 수 있다. 따라서 장애 분석에서는 잘못된 로봇 포즈와 오래되었거나 부분적으로 손상된 지도를 구분해야 한다.

잘못된 측정값이 반복적으로 상태 추정 과정에 입력되면 상태 추정기 발산(Estimator Divergence)이 발생할 수 있다. 잘못된 루프 폐쇄(False Loop Closure), 부정확한 스캔 정합, 과소평가된 센서 공분산(Sensor Covariance), 과도한 관성 바이어스(Inertial Bias), 일관되지 않은 제약조건 등이 궤적을 점진적으로 왜곡할 수 있다. 혁신값(Innovation), 정합 잔차(Registration Residual), 공분산, 최적화 비용(Optimization Cost), 제약조건 일관성을 모니터링하면 상태 추정 결과가 심각하게 잘못되기 전에 발산을 식별하는 데 도움이 된다.

루프 폐쇄 장애(Loop-closure Failure)는 잘못된 하나의 폐쇄 결과가 지도나 궤적의 상당 부분을 변경할 수 있으므로 특별한 주의가 필요하다. 반복적인 환경에서는 서로 다른 두 장소가 기하학적 또는 시각적으로 유사하게 보일 수 있다. 따라서 후보 루프 폐쇄는 승인 전에 기하학적 검증(Geometric Verification)과 일관성 검사를 통과해야 한다. 새롭게 추가된 루프 제약조건으로 큰 보정값이 발생하는 경우에도 이를 자동으로 신뢰하기보다 추가적인 검증을 수행해야 한다.

센싱과 알고리즘이 정상이어도 연산 시스템(Computing System)에서 장애가 발생할 수 있다. CPU 또는 GPU 포화, 메모리 부족, 열 스로틀링(Thermal Throttling), 통신 혼잡, 저장장치 지연, 미들웨어 과부하는 프레임 누락과 불규칙한 처리 지연시간을 발생시킬 수 있다. 오래된 센서 데이터로 동작하는 SLAM은 센서 장애와 유사한 위치추정 오차를 생성할 수 있으므로 자원 사용률과 종단간 처리 시간(End-to-end Timing)도 필수적인 진단 신호이다.

장애 검출(Failure Detection)은 하나의 임계값에 의존하기보다 여러 지표를 결합해야 한다. 스캔 정합 점수, 재투영 오차(Reprojection Error), 특징점 수, IMU 잔차, 휠 일관성, 포즈 공분산(Pose Covariance), 지도 중첩도(Map Overlap), 루프 폐쇄 신뢰도, 처리 지연시간, 센서 상태 등을 함께 이용하여 위치추정 품질을 판단할 수 있다. 상태 관리자(Health Manager)는 이러한 신호를 종합하여 정상(Normal), 성능 저하(Degraded), 위험(Critical), 위치추정 상실(Lost Localization)과 같은 운영 상태를 결정할 수 있다.

성능 저하 상태는 위치추정이 완전히 상실되기 전에 로봇 행동에 영향을 주어야 한다. 로봇은 속도를 낮추고, 장애물과의 안전 여유를 증가시키며, 좁은 통로를 회피하고, 정밀 도킹(Precision Docking)을 중단하거나, 위치추정 특징이 더욱 풍부한 영역으로 이동할 수 있다. 이러한 행동은 불확실성으로 인한 영향을 제한하면서 SLAM 시스템이 복구에 필요한 추가 시간이나 더 좋은 관측 조건을 확보하도록 한다.

신뢰도가 안전 임계값(Safe Threshold) 이하로 떨어지면 자율주행을 계속하기보다 통제된 정지(Controlled Stop)를 수행하는 것이 바람직한 경우가 많다. 가능한 경우 로봇은 안정적인 상태로 정지하고 최근의 센서 및 상태 추정 데이터를 보존하며, 마지막으로 추정된 포즈가 계속 유효하다고 내비게이션 시스템이 가정하지 못하도록 해야 한다. 안전 동작은 SLAM 장애가 위험한 움직임을 방지하는 메커니즘까지 무력화하지 않도록 충분히 독립적으로 유지되어야 한다.

재위치추정(Relocalization)은 포즈 추적이 상실된 이후의 주요 복구 메커니즘이다. 현재 라이다 스캔 또는 시각 관측값을 기존에 저장된 지도, 서브맵(Submap), 키프레임(Keyframe), 장소 기술자(Place Descriptor)와 비교할 수 있다. 초기에는 여러 개의 후보 포즈를 생성할 수 있으며, 기하학적 검증을 통해 현재 측정값과 일치하는 가설을 결정할 수 있다. 포즈 신뢰도가 충분히 확보된 이후에만 자율 내비게이션을 재개해야 한다.

복구 과정에서는 최근의 이동 궤적 이력(Trajectory History)을 활용할 수도 있다. 어려운 영역에 진입한 이후 위치추정을 상실했다면 로봇은 신뢰할 수 있는 추적이 마지막으로 가능했던 위치에 대한 단기적인 추정값을 유지할 수 있다. 해당 영역으로 후퇴(Backtracking)하면 지도 중첩이나 환경 특징을 다시 확보할 수 있으며, 이러한 전략은 반복적인 복도, 임시 공사 영역, 식생 또는 센서 가시성이 갑자기 악화되는 위치에서 특히 유용하다.

다중 센서 시스템(Multi-sensor System)은 센서 가중치 또는 위치추정 모드(Localization Mode)를 변경하여 부분적인 장애 상황에서도 운용을 지속할 수 있다. 라이다 장애가 발생하면 시각-관성 상태 추정(Visual-Inertial Estimation)의 비중을 높일 수 있고, 조명 조건이 나빠지면 라이다와 IMU에 더 의존할 수 있다. 휠 오도메트리, GNSS, 레이더, 깊이 센싱, 인프라 랜드마크도 추가적인 제약조건을 제공할 수 있지만 모드 전환 과정에서 갑작스러운 좌표 변화나 근거 없는 신뢰도 증가가 발생해서는 안 된다.

복구 관리자(Recovery Manager)는 일시적인 성능 저하와 지속적인 하드웨어 또는 지도 장애를 구분해야 한다. 특정 위치에서 재위치추정 실패가 반복된다면 환경 변화가 원인일 수 있으며, 하나의 로봇이 여러 장소에서 지속적으로 장애를 경험한다면 센서나 보정 문제가 원인일 가능성이 높다. 다른 로봇의 관측값을 비교하면 장애가 특정 위치에 국한된 것인지 특정 플랫폼에 국한된 것인지 판단할 수 있으므로 플릿 수준 비교(Fleet-level Comparison)가 유용하다.

장애 격리(Fault Isolation)는 하나의 고장 구성요소가 전체 상태 추정 체인을 손상시키는 것을 방지한다. 비정상적인 타임스탬프, 물리적으로 타당하지 않은 움직임, 과도한 잔차, 일관되지 않은 공분산을 가진 측정값은 제거하거나 가중치를 낮출 수 있다. 특정 센서가 신뢰할 수 없는 것으로 판단되면 상태 추정기에서 일시적으로 제외하면서도 유지보수 및 이후 근본 원인 분석(Root-cause Analysis)을 위한 진단 정보는 보존해야 한다.

장애 이후 분석(Post-failure Analysis)을 위해서는 서로 분리된 오류 메시지가 아니라 동기화된 로그(Synchronized Log)가 필요하다. 센서 데이터, 추정 포즈, 좌표 변환(Transform), 공분산, 최적화 잔차, 루프 폐쇄, CPU 및 GPU 부하, 네트워크 지연, 지도 버전, 소프트웨어 설정, 내비게이션 명령이 일관된 시간축을 공유해야 한다. 이를 통해 엔지니어는 서로 단절된 하위 시스템 기록을 개별적으로 분석하는 대신 위치추정 성능 저하 전후의 전체 과정을 재구성할 수 있다.

근본 원인 분석은 최초의 원인 사건(Initiating Event)과 이후 발생한 2차 결과(Secondary Consequence)를 구분해야 한다. 예를 들어 진동이 IMU 측정값을 손상시키고, 이것이 스캔 디스큐(Scan Deskewing) 오류를 발생시키며, 이후 라이다 정합 품질을 감소시키고 최종적으로 내비게이션 진동을 유발할 수 있다. 최종적인 내비게이션 진동을 최초 장애로 판단하면 효과적인 개선 조치를 수행하기 어렵기 때문에 인과관계 시간선(Causal Timeline)을 구성하는 것이 중요하다.

복구 성능(Recovery Performance)은 명시적으로 측정해야 한다. 성능 저하 검출 시간(Time to Detect Degradation), 안전 상태 전환 시간(Time to Safe State), 재위치추정 시간(Relocalization Time), 복구 성공률, 성능 저하 상태에서 이동한 거리, 복구 이후 포즈 오차, 작업자 개입 횟수 등이 유용한 지표가 된다. 이러한 측정값을 이용하면 서로 다른 복구 전략을 비교하고 소프트웨어 업데이트가 실제로 시스템 복원력(Resilience)을 향상시키는지 평가할 수 있다.

검증(Validation)에서는 현실적인 장애 조건을 의도적으로 주입해야 한다. 센서 연결 해제, 지연된 타임스탬프, 라이다 반사값 저하, 카메라 가림, IMU 진동, 휠 슬립(Wheel Slip), 잘못된 루프 폐쇄 후보, 오래된 지도, 프로세서 과부하, 통신 단절, 급격한 환경 변화 등을 시험할 수 있다. 목표는 정상 상태의 SLAM 정확도만 입증하는 것이 아니라 시스템의 기본 가정이 무너졌을 때에도 예측 가능한 방식으로 동작하는지를 검증하는 것이다.

플릿 운용(Fleet Operation)은 추가적인 복구 자원을 제공한다. 위치추정에 실패한 로봇은 다른 로봇이 확보한 검증된 지도 영역, 최근 환경 관측값 또는 인프라 정보를 요청할 수 있다. 또한 플릿 모니터링을 통해 동일한 위치에서 반복적으로 발생하는 장애를 검출하고, 지도 또는 인프라 개선이 완료될 때까지 해당 영역을 위치추정 위험 구역(Localization Risk Zone)으로 분류할 수 있다.

성숙한 SLAM 장애 관리 아키텍처(SLAM Failure-management Architecture)는 상태 모니터링(Health Monitoring), 성능 저하 검출(Degradation Detection), 안전 상태 전환(Safe-state Transition), 장애 격리, 재위치추정, 복구 검증(Recovery Validation), 근본 원인 분석으로 이어지는 폐루프 구조(Closed Loop)를 형성한다. 신뢰할 수 있는 자율성(Reliable Autonomy)은 SLAM이 절대로 실패하지 않는 것을 의미하지 않는다. 중요한 것은 장애를 관측 가능하고, 영향 범위를 제한할 수 있으며, 진단 가능하고, 복구 가능한 상태로 만들어 위치추정 불확실성이 로봇 행동 전체로 통제되지 않은 채 확산되는 것을 방지하는 것이다.

##  

## 12.10. Future SLAM and Localization Roadmap

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

Future SLAM and localization systems will evolve from isolated geometric estimators into integrated spatial-intelligence systems that support perception, prediction, planning, and autonomous decision-making. Classical localization will remain fundamental, but future robots will increasingly require maps that represent not only where objects and structures exist, but also what they are, how they change, and how they influence possible actions.

The near-term roadmap will continue improving multi-sensor localization through tighter integration of LiDAR, cameras, IMU, wheel odometry, GNSS, radar, depth sensing, and infrastructure references. Rather than depending on one dominant sensor, production systems will dynamically exploit complementary sensing characteristics so that localization remains available across lighting changes, weather, vibration, repetitive geometry, and temporary sensor degradation.

Sensor fusion will become increasingly adaptive instead of relying on fixed measurement weights and static noise assumptions. Localization systems can evaluate sensor quality, environmental observability, residual consistency, and operating context before determining how strongly each measurement should influence the state estimate. This allows the estimator to reduce dependence on sensors that are temporarily unreliable without immediately losing localization.

Three-dimensional localization will become increasingly important as autonomous systems expand beyond conventional planar AMRs. Outdoor robots, quadrupeds, humanoids, mobile manipulators, autonomous vehicles, and UAVs require full six-degree-of-freedom spatial reasoning. Future mapping architectures will therefore combine metric geometry, traversability, elevation, free space, structural information, and platform-specific motion constraints within common spatial representations.

Semantic SLAM will extend maps beyond points, grids, and surfaces. Robots will associate doors, shelves, machines, roads, stairs, elevators, workstations, people, loading zones, vegetation, and other meaningful entities with spatial coordinates. Such maps allow autonomy systems to reason about task-relevant places and objects while preserving the geometric accuracy required for navigation, manipulation, inspection, and safety.

Object-level mapping will further change the role of SLAM. Instead of representing every environment primarily as static geometry, robots can maintain persistent objects with identity, pose, dimensions, semantic class, and state. A robot may understand that a pallet moved, a door opened, or a machine changed configuration without interpreting every difference as corruption of the underlying geometric map.

Dynamic SLAM will become necessary as robots operate continuously in environments shared with people, vehicles, machinery, and other robots. Future estimators will explicitly distinguish static structure from moving or temporarily displaced objects. Tracking and motion prediction can allow dynamic entities to remain part of the world model without incorrectly becoming permanent localization landmarks.

Long-term autonomy will require maps that continuously evolve while preserving stable spatial references. Instead of periodically replacing one monolithic map with another, future systems can maintain persistent structural layers, dynamic layers, semantic layers, and historical changes. Map versioning, confidence, provenance, validation, and rollback will become normal infrastructure functions rather than secondary SLAM engineering tasks.

Learning-based methods will increasingly complement geometric estimation. Neural networks can improve feature extraction, place recognition, depth estimation, semantic understanding, dynamic-object filtering, and correspondence generation. However, production localization will continue to benefit from explicit geometric constraints because metric consistency, uncertainty, diagnosability, and predictable failure behavior remain essential for safety-critical autonomous systems.

The most practical architectures are therefore likely to combine learned perception with geometric optimization rather than replacing geometry completely. Learned components can propose correspondences, landmarks, descriptors, depth, or semantic relationships, while factor graphs, filtering, bundle adjustment, and registration provide physically interpretable spatial constraints. This hybrid approach combines adaptability with measurable geometric consistency.

Foundation models may extend spatial perception toward generalized scene understanding. Large visual and multimodal models can potentially recognize unfamiliar objects, interpret signs, understand functional regions, and connect language instructions with mapped locations. Localization can then become part of a broader reasoning system in which the robot understands both its metric position and the operational meaning of its surroundings.

World models will push this development further by representing how environments change over time and how robot actions influence future states. A spatial map describes the current or accumulated environment, while a world model can predict likely transitions. Combining localization with temporal prediction may allow robots to reason about moving obstacles, changing accessibility, human activity, and the consequences of alternative navigation actions.

Active localization will also become more important. Instead of passively accepting whatever observations occur along a planned trajectory, robots can deliberately move sensors or alter their path to reduce localization uncertainty. A humanoid may rotate its head toward useful landmarks, an AMR may slightly adjust its route, and a UAV may change viewpoint to recover stronger geometric constraints before entering a difficult region.

Localization uncertainty will increasingly become a first-class input to planning. Future autonomy stacks should not treat the estimated pose as an exact value. Route selection, velocity, obstacle clearance, docking strategy, manipulation precision, and mission decisions can adapt according to covariance, observability, map confidence, and sensor health. This connects estimation quality directly with operational risk.

Collaborative localization will expand from individual robots to fleets. Multiple robots can exchange submaps, place descriptors, environmental changes, localization confidence, and validated landmarks. A robot entering a weakly observable region may benefit from information collected earlier by another platform, while fleet statistics can distinguish location-specific mapping problems from faults affecting only one robot.

Heterogeneous collaborative mapping will become particularly valuable. AMRs, quadrupeds, humanoids, UAVs, and fixed infrastructure sensors observe the environment from different heights, viewpoints, and sensing modalities. Combining these observations can create richer shared maps than any single platform can generate, provided that coordinate alignment, confidence management, data provenance, and sensor-specific uncertainty are maintained.

Cloud and on-premise infrastructure can support computationally expensive global optimization, map consolidation, historical analysis, and fleet-wide learning, while edge computers maintain low-latency local localization. This separation enables robots to remain operational during network interruptions while still benefiting from larger computational resources when connectivity is available. Local autonomy and global spatial intelligence can therefore coexist.

Map distribution will increasingly resemble managed software deployment. Spatial representations will require version identifiers, compatibility information, integrity verification, staged release, rollback, and monitoring. Different robots may require different map resolutions or layers while still sharing a common global coordinate reference. Efficient selective distribution will become essential as maps and fleets grow larger.

Future localization systems will incorporate stronger self-diagnosis. Rather than simply reporting a pose until tracking fails, estimators will monitor observability, sensor consistency, residual patterns, computational latency, calibration health, and map agreement. These signals can support early degradation detection and allow autonomous systems to enter reduced-performance or recovery modes before localization becomes completely unreliable.

Automatic recovery will become an expected capability rather than an exceptional procedure. Robots will attempt relocalization, viewpoint change, backtracking, sensor-mode switching, fleet-assisted recovery, or navigation toward better-observed regions. Recovery success must be verified through independent measurements before normal autonomy resumes, ensuring that an apparently plausible but incorrect pose does not silently re-enter the control system.

Safety architectures will increasingly separate localization performance from localization trust. An estimator may continue producing numerical poses while the safety system independently evaluates whether those estimates are sufficiently reliable for the current maneuver. High-speed motion, narrow passages, precision docking, manipulation, and flight may require different localization-confidence thresholds because the consequence of position error varies with the task.

Simulation and digital twins will become central to localization development and validation. Large combinations of sensor faults, environmental changes, weather, lighting, dynamic objects, map errors, communication loss, and computational degradation can be tested before deployment. Real-world logs can then reproduce difficult failures in simulation, allowing localization algorithms and recovery logic to be evaluated systematically.

Continuous validation will replace the assumption that localization is fully verified at commissioning. Fleet telemetry can reveal new failure modes, environmental weak points, calibration drift, and map aging throughout operation. These observations can feed regression testing and controlled updates so that improvements are evaluated against both historical failures and representative current operating conditions.

Standardized interfaces between localization, mapping, planning, safety, and fleet management will become increasingly important as autonomy systems grow more complex. Pose alone is insufficient. Downstream systems also need uncertainty, reference-frame identity, map version, localization mode, sensor health, degradation state, and timestamp information to interpret spatial estimates correctly and make appropriate operational decisions.

The long-term direction is toward persistent spatial intelligence rather than standalone SLAM. Geometry, semantics, objects, dynamics, uncertainty, historical change, and predictive models will increasingly coexist within a shared representation. Different autonomous platforms can consume different portions of this representation while maintaining a consistent understanding of location and environmental structure.

Ultimately, future SLAM and localization will become an enabling layer for Physical AI rather than a self-contained navigation technology. Reliable spatial estimation will connect perception with world models, reasoning, planning, manipulation, locomotion, fleet coordination, and safety. The roadmap therefore progresses from accurate pose estimation toward continuously updated, uncertainty-aware, semantic, collaborative, and predictive spatial intelligence that supports long-term autonomous operation.

미래의 SLAM 및 위치추정(Localization) 시스템은 독립적인 기하학적 상태 추정기(Geometric Estimator)에서 인식, 예측, 계획, 자율 의사결정을 지원하는 통합 공간 지능 시스템(Spatial-intelligence System)으로 발전할 것이다. 전통적인 위치추정은 계속 핵심 기반 기술로 남겠지만, 미래의 로봇은 객체와 구조물이 어디에 존재하는지만 표현하는 지도가 아니라 그것이 무엇인지, 어떻게 변화하는지, 가능한 행동에 어떤 영향을 미치는지까지 표현하는 지도를 점점 더 필요로 하게 될 것이다.

단기적인 로드맵(Near-term Roadmap)에서는 라이다(LiDAR), 카메라, 관성측정장치(IMU), 휠 오도메트리(Wheel Odometry), GNSS, 레이더(Radar), 깊이 센싱(Depth Sensing), 인프라 기준(Infrastructure Reference)을 더욱 긴밀하게 통합하여 다중 센서 위치추정(Multi-sensor Localization)을 지속적으로 개선하게 될 것이다. 하나의 지배적인 센서에 의존하기보다 생산 시스템은 상호 보완적인 센싱 특성을 동적으로 활용하여 조명 변화, 날씨, 진동, 반복적인 기하학 구조, 일시적인 센서 성능 저하에서도 위치추정을 유지하게 될 것이다.

센서 융합(Sensor Fusion)은 고정된 측정 가중치와 정적인 노이즈 가정에 의존하는 방식에서 점차 적응형(Adaptive) 방식으로 발전할 것이다. 위치추정 시스템은 각 측정값이 상태 추정에 얼마나 강하게 영향을 주어야 하는지 결정하기 전에 센서 품질, 환경 관측 가능성(Environmental Observability), 잔차 일관성(Residual Consistency), 운용 상황을 평가할 수 있다. 이를 통해 일시적으로 신뢰성이 떨어진 센서에 대한 의존도를 낮추면서도 위치추정을 즉시 상실하지 않을 수 있다.

자율 시스템이 기존의 평면형 AMR을 넘어 확장되면서 3차원 위치추정(Three-dimensional Localization)의 중요성은 더욱 증가할 것이다. 실외 로봇, 4족 보행 로봇(Quadruped), 휴머노이드(Humanoid), 모바일 매니퓰레이터(Mobile Manipulator), 자율주행 차량, UAV는 완전한 6자유도 공간 추론(6-DoF Spatial Reasoning)을 필요로 한다. 따라서 미래의 지도작성 아키텍처는 기하학적 계측 정보, 주행 가능성(Traversability), 고도, 자유 공간, 구조 정보, 플랫폼별 운동 제약조건을 공통 공간 표현(Common Spatial Representation) 안에서 결합하게 될 것이다.

의미론적 SLAM(Semantic SLAM)은 지도를 점, 격자, 표면 이상의 표현으로 확장할 것이다. 로봇은 문, 선반, 기계, 도로, 계단, 엘리베이터, 작업대, 사람, 적재 구역, 식생 및 기타 의미 있는 객체를 공간 좌표와 연결하게 된다. 이러한 지도는 내비게이션, 조작, 점검, 안전에 필요한 기하학적 정확성을 유지하면서 자율 시스템이 임무와 관련된 장소와 객체에 대해 추론할 수 있도록 한다.

객체 수준 지도작성(Object-level Mapping)은 SLAM의 역할을 더욱 변화시킬 것이다. 모든 환경을 주로 정적인 기하학 구조로 표현하는 대신 로봇은 식별 정보(Identity), 포즈, 크기, 의미론적 클래스(Semantic Class), 상태를 가진 지속적인 객체(Persistent Object)를 관리할 수 있다. 이를 통해 로봇은 모든 차이를 기본 기하학 지도의 손상으로 해석하지 않고 팔레트가 이동했거나, 문이 열렸거나, 기계의 구성이 변경되었다는 사실을 이해할 수 있다.

동적 SLAM(Dynamic SLAM)은 로봇이 사람, 차량, 기계, 다른 로봇과 지속적으로 공간을 공유하게 되면서 필수적인 기술이 될 것이다. 미래의 상태 추정기는 정적인 구조물과 이동하거나 일시적으로 위치가 변경된 객체를 명시적으로 구분하게 된다. 객체 추적(Object Tracking)과 움직임 예측(Motion Prediction)을 통해 동적 객체를 영구적인 위치추정 랜드마크로 잘못 사용하는 대신 세계 모델(World Model)의 일부로 유지할 수 있다.

장기 자율성(Long-term Autonomy)을 위해서는 안정적인 공간 기준을 유지하면서 지속적으로 변화하는 지도가 필요하다. 하나의 거대한 지도를 주기적으로 다른 지도로 교체하는 대신 미래 시스템은 지속적인 구조 계층(Persistent Structural Layer), 동적 계층(Dynamic Layer), 의미론적 계층(Semantic Layer), 과거 변화 이력(Historical Change)을 함께 관리할 수 있다. 지도 버전 관리, 신뢰도, 데이터 출처 추적(Provenance), 검증, 롤백(Rollback)은 부수적인 SLAM 엔지니어링 작업이 아니라 일반적인 인프라 기능이 될 것이다.

학습 기반 방법(Learning-based Method)은 기하학적 상태 추정을 점점 더 보완하게 될 것이다. 신경망(Neural Network)은 특징 추출, 장소 인식(Place Recognition), 깊이 추정, 의미론적 이해, 동적 객체 필터링, 대응 관계 생성(Correspondence Generation)을 향상시킬 수 있다. 그러나 생산 환경의 위치추정에서는 계측적 일관성(Metric Consistency), 불확실성, 진단 가능성(Diagnosability), 예측 가능한 장애 동작이 안전 중심 자율 시스템에서 필수적이므로 명시적인 기하학적 제약조건이 계속 중요한 역할을 하게 될 것이다.

따라서 가장 실용적인 아키텍처는 기하학을 완전히 대체하기보다 학습 기반 인식(Learned Perception)과 기하학적 최적화(Geometric Optimization)를 결합하는 형태가 될 가능성이 높다. 학습 기반 구성요소는 대응점, 랜드마크, 기술자(Descriptor), 깊이 또는 의미론적 관계를 제안하고, 팩터 그래프(Factor Graph), 필터링, 번들 조정(Bundle Adjustment), 정합(Registration)은 물리적으로 해석 가능한 공간 제약조건을 제공할 수 있다. 이러한 하이브리드 접근법(Hybrid Approach)은 적응성과 측정 가능한 기하학적 일관성을 결합한다.

파운데이션 모델(Foundation Model)은 공간 인식을 일반화된 장면 이해(Generalized Scene Understanding)로 확장할 수 있다. 대규모 시각 및 멀티모달 모델(Multimodal Model)은 익숙하지 않은 객체를 인식하고, 표지판을 해석하며, 기능적 영역을 이해하고, 자연어 명령과 지도상의 위치를 연결할 수 있다. 이에 따라 위치추정은 로봇이 자신의 계측적 위치뿐만 아니라 주변 환경의 운영적 의미까지 이해하는 더 광범위한 추론 시스템(Reasoning System)의 일부가 될 수 있다.

월드 모델(World Model)은 환경이 시간에 따라 어떻게 변화하고 로봇의 행동이 미래 상태에 어떤 영향을 주는지를 표현함으로써 이러한 발전을 더욱 확장할 것이다. 공간 지도(Spatial Map)가 현재 또는 누적된 환경을 기술한다면 월드 모델은 가능한 상태 전이(State Transition)를 예측할 수 있다. 위치추정과 시간적 예측(Temporal Prediction)을 결합하면 이동 장애물, 접근 가능성 변화, 사람의 활동, 서로 다른 내비게이션 행동의 결과를 로봇이 추론할 수 있게 된다.

능동 위치추정(Active Localization)의 중요성도 더욱 증가할 것이다. 로봇은 계획된 궤적을 따라 이동하면서 우연히 얻어지는 관측값만 수동적으로 사용하는 대신 위치추정 불확실성을 줄이기 위해 의도적으로 센서를 움직이거나 경로를 변경할 수 있다. 휴머노이드는 유용한 랜드마크 방향으로 머리를 회전하고, AMR은 경로를 약간 수정하며, UAV는 어려운 영역에 진입하기 전에 더 강한 기하학적 제약조건을 확보하기 위해 관측 시점(Viewpoint)을 변경할 수 있다.

위치추정 불확실성(Localization Uncertainty)은 점차 계획 시스템의 핵심 입력값(First-class Input)이 될 것이다. 미래의 자율 시스템은 추정된 포즈를 정확한 단일 값으로 취급해서는 안 된다. 경로 선택, 속도, 장애물 여유 거리, 도킹 전략, 조작 정밀도, 임무 의사결정은 공분산(Covariance), 관측 가능성, 지도 신뢰도(Map Confidence), 센서 상태에 따라 적응할 수 있다. 이를 통해 상태 추정 품질이 운영 위험과 직접적으로 연결된다.

협업 위치추정(Collaborative Localization)은 개별 로봇에서 플릿(Fleet)으로 확장될 것이다. 여러 로봇은 서브맵(Submap), 장소 기술자(Place Descriptor), 환경 변화, 위치추정 신뢰도, 검증된 랜드마크를 서로 교환할 수 있다. 관측 가능성이 낮은 영역에 진입하는 로봇은 이전에 다른 플랫폼이 수집한 정보를 활용할 수 있으며, 플릿 통계는 특정 위치의 지도 문제와 개별 로봇에서만 발생하는 장애를 구분하는 데 활용될 수 있다.

이종 플랫폼 협업 지도작성(Heterogeneous Collaborative Mapping)은 특히 높은 가치를 제공하게 될 것이다. AMR, 4족 보행 로봇, 휴머노이드, UAV, 고정형 인프라 센서는 서로 다른 높이, 관측 시점, 센싱 방식에서 환경을 관측한다. 좌표 정렬, 신뢰도 관리, 데이터 출처 추적, 센서별 불확실성을 유지할 수 있다면 이러한 관측값을 결합하여 단일 플랫폼만으로 생성하기 어려운 더욱 풍부한 공유 지도(Shared Map)를 구축할 수 있다.

클라우드(Cloud) 및 온프레미스(On-premise) 인프라는 계산량이 많은 전역 최적화(Global Optimization), 지도 통합(Map Consolidation), 과거 데이터 분석, 플릿 전체 학습을 지원하고, 엣지 컴퓨터(Edge Computer)는 낮은 지연시간의 지역 위치추정을 유지할 수 있다. 이러한 역할 분리를 통해 로봇은 네트워크 단절 중에도 계속 운용되면서 연결이 가능한 경우 더 큰 연산 자원을 활용할 수 있다. 따라서 지역 자율성(Local Autonomy)과 전역 공간 지능(Global Spatial Intelligence)이 공존할 수 있다.

지도 배포(Map Distribution)는 점차 관리형 소프트웨어 배포(Managed Software Deployment)와 유사해질 것이다. 공간 표현에는 버전 식별자, 호환성 정보, 무결성 검증(Integrity Verification), 단계적 배포(Staged Release), 롤백, 모니터링이 필요하게 된다. 서로 다른 로봇은 서로 다른 지도 해상도 또는 계층을 필요로 할 수 있지만 동일한 전역 좌표 기준(Global Coordinate Reference)을 공유할 수 있다. 지도와 플릿 규모가 증가함에 따라 효율적인 선택적 배포(Selective Distribution)가 필수적이 될 것이다.

미래의 위치추정 시스템은 더욱 강력한 자가 진단(Self-diagnosis) 기능을 포함하게 될 것이다. 추적이 실패할 때까지 단순히 포즈만 출력하는 대신 상태 추정기는 관측 가능성, 센서 일관성, 잔차 패턴, 연산 지연시간, 보정 상태(Calibration Health), 지도 일치도를 지속적으로 모니터링하게 된다. 이러한 신호는 조기 성능 저하 검출(Early Degradation Detection)을 지원하고 위치추정이 완전히 불가능해지기 전에 자율 시스템이 성능 제한 모드 또는 복구 모드(Recovery Mode)로 전환하도록 할 수 있다.

자동 복구(Automatic Recovery)는 예외적인 절차가 아니라 기본적으로 기대되는 기능이 될 것이다. 로봇은 재위치추정(Relocalization), 관측 시점 변경, 후퇴(Backtracking), 센서 모드 전환, 플릿 지원 복구(Fleet-assisted Recovery), 관측 조건이 더 좋은 영역으로의 이동 등을 시도하게 된다. 정상적인 자율 동작을 재개하기 전에 독립적인 측정값을 통해 복구 성공 여부를 검증하여 그럴듯해 보이지만 잘못된 포즈가 제어 시스템에 다시 입력되는 것을 방지해야 한다.

안전 아키텍처(Safety Architecture)는 위치추정 성능(Localization Performance)과 위치추정 신뢰(Localization Trust)를 점차 분리하게 될 것이다. 상태 추정기가 계속 수치적인 포즈를 출력하더라도 안전 시스템은 해당 추정값이 현재 수행 중인 기동에 충분히 신뢰할 수 있는지를 독립적으로 평가할 수 있다. 고속 이동, 좁은 통로, 정밀 도킹, 조작, 비행은 위치 오차가 초래하는 결과가 서로 다르기 때문에 서로 다른 위치추정 신뢰도 임계값을 요구할 수 있다.

시뮬레이션(Simulation)과 디지털 트윈(Digital Twin)은 위치추정 개발 및 검증에서 핵심적인 역할을 하게 될 것이다. 센서 장애, 환경 변화, 날씨, 조명, 동적 객체, 지도 오류, 통신 단절, 연산 성능 저하의 다양한 조합을 실제 배치 전에 시험할 수 있다. 실제 환경에서 수집한 로그를 이용하여 어려운 장애 상황을 시뮬레이션에서 재현하고 위치추정 알고리즘과 복구 로직을 체계적으로 평가할 수 있다.

지속적 검증(Continuous Validation)은 위치추정이 초기 구축 단계에서 완전히 검증된다는 기존의 가정을 대체하게 될 것이다. 플릿 텔레메트리(Fleet Telemetry)는 운용 기간 전체에 걸쳐 새로운 장애 유형, 환경 취약 지점, 보정 드리프트(Calibration Drift), 지도 노후화(Map Aging)를 발견할 수 있다. 이러한 관측값은 회귀 시험(Regression Testing)과 통제된 업데이트에 활용되어 개선 사항을 과거 장애 사례와 현재의 대표적인 운용 조건 모두에 대해 평가할 수 있도록 한다.

자율 시스템이 복잡해질수록 위치추정, 지도작성, 계획, 안전, 플릿 관리 사이의 표준화된 인터페이스(Standardized Interface)가 더욱 중요해질 것이다. 포즈 정보만으로는 충분하지 않다. 하위 시스템이 공간 추정값을 올바르게 해석하고 적절한 운영 의사결정을 수행하려면 불확실성, 기준 좌표 프레임 식별 정보, 지도 버전, 위치추정 모드, 센서 상태, 성능 저하 상태, 타임스탬프 정보도 함께 제공되어야 한다.

장기적인 발전 방향은 독립적인 SLAM을 넘어 지속적인 공간 지능(Persistent Spatial Intelligence)으로 향하게 될 것이다. 기하학, 의미론(Semantics), 객체, 동적 정보, 불확실성, 과거 변화, 예측 모델(Predictive Model)이 하나의 공유 표현 안에서 점차 공존하게 된다. 서로 다른 자율 플랫폼은 이 표현의 서로 다른 부분을 활용하면서도 위치와 환경 구조에 대한 일관된 이해를 유지할 수 있다.

궁극적으로 미래의 SLAM 및 위치추정은 독립적인 내비게이션 기술이 아니라 피지컬 AI(Physical AI)를 구현하는 핵심 기반 계층(Enabling Layer)이 될 것이다. 신뢰할 수 있는 공간 상태 추정은 인식(Perception)을 월드 모델, 추론(Reasoning), 계획, 조작, 이동(Locomotion), 플릿 조정(Fleet Coordination), 안전과 연결한다. 따라서 미래 로드맵은 정확한 포즈 추정에서 출발하여 지속적으로 갱신되고, 불확실성을 인식하며, 의미론적이고, 협업 가능하며, 미래를 예측하는 공간 지능으로 발전함으로써 장기적인 자율 운용을 지원하는 방향으로 진행될 것이다.
