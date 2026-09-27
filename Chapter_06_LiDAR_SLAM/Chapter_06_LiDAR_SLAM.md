**Volume 13. Mapping Localization and SLAM**


# Chapter 06. LiDAR SLAM

##  

## 06.01. LiDAR SLAM Pipeline Scan Registration Map Update

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

LiDAR SLAM is a recursive estimation process that converts sequential laser scans into a consistent representation of robot motion and the surrounding environment. Its core pipeline typically consists of sensor acquisition, preprocessing, motion compensation, scan registration, pose estimation, map update, and optimization. Each stage transforms raw geometric observations into increasingly structured spatial information while maintaining uncertainty about the robot state and accumulated map.

A LiDAR sensor measures distances by emitting laser energy and observing its return from surrounding surfaces. A single acquisition produces a scan represented as ranges, angles, or three-dimensional points depending on the sensor architecture. Before registration, invalid returns, extremely near or distant measurements, isolated outliers, and redundant points are commonly removed. Voxel filtering or geometric sampling can reduce computational load while preserving structures useful for localization.

Real LiDAR scans are not necessarily instantaneous measurements. Mechanical spinning sensors collect points over a finite interval while the robot may simultaneously translate and rotate. This creates motion distortion, particularly on fast-moving outdoor AMRs or vehicles. Deskewing compensates individual points using timestamps and estimated motion obtained from an IMU, wheel odometry, or previous LiDAR states, producing a geometrically coherent scan referenced to a common time.

The registration stage estimates the rigid transformation that best aligns the current scan with previously observed geometry. Scan-to-scan registration compares consecutive measurements, whereas scan-to-map registration aligns new observations against an accumulated local or global map. The estimated transformation normally contains three-dimensional translation and rotation, providing the relative motion constraint required to propagate the robot pose through the SLAM trajectory.

Iterative Closest Point, or ICP, is a fundamental registration method that repeatedly establishes correspondences between two point sets and minimizes their geometric discrepancy. Point-to-point ICP minimizes Euclidean distances, while point-to-plane formulations exploit local surface normals and often converge more effectively in structured environments. ICP is conceptually simple, but its performance depends strongly on initialization, correspondence quality, environmental geometry, and rejection of dynamic objects.

Normal Distributions Transform, or NDT, represents reference points using probability distributions over spatial cells rather than requiring explicit point-to-point correspondences. Registration evaluates how well transformed scan points agree with these distributions and iteratively optimizes the transformation. NDT can provide efficient alignment for large point clouds and has been widely applied to vehicle localization, although voxel resolution and optimization parameters must be matched to sensor density and environmental scale.

Modern LiDAR SLAM systems frequently extract geometric features before registration. Planar regions, edges, surface patches, or locally distinctive structures can provide stronger constraints than indiscriminately processing every point. Feature selection reduces data volume and emphasizes stable geometry. However, feature-based approaches may degrade in environments where the expected structures are sparse, so many systems combine feature extraction with direct scan matching or adaptive geometric representations.

Registration requires a sufficiently accurate initial pose estimate because nonlinear optimization can converge to an incorrect local solution when displacement between scans is large. Wheel odometry, inertial measurements, GNSS, constant-velocity models, or the previous SLAM state can supply this prediction. LiDAR registration then corrects accumulated motion errors. This prediction-correction relationship is especially important for high-speed robots, rough terrain, rapid rotations, and scenes with limited geometric constraints.

After registration, the estimated relative transformation is integrated into the robot trajectory. The current pose can be expressed as a composition of the previous pose, predicted motion, and registration correction. Practical systems also estimate registration confidence using residual error, correspondence statistics, geometric degeneracy, or covariance approximations. Poorly constrained directions must be recognized because apparently successful alignment can still contain significant uncertainty along corridors, tunnels, or large planar surfaces.

Map update determines how newly registered measurements become part of the spatial representation. A simple point-cloud map inserts transformed points directly into a global coordinate frame, but unrestricted accumulation rapidly increases memory and computation. Voxel grids, octrees, surfels, probability distributions, or hierarchical submaps therefore provide more scalable representations. The choice depends on whether the map is intended primarily for localization, navigation, obstacle reasoning, visualization, or long-term operation.

Many real-time systems maintain a local map containing only geometry surrounding the current robot position. Incoming scans are registered against this local representation, and old regions are removed or transferred into larger submaps as the robot moves. This limits computational complexity and keeps registration focused on relevant geometry. Local mapping is particularly effective for outdoor AMRs operating over large facilities, roads, industrial sites, ports, campuses, or logistics environments.

A critical design issue is deciding which registered scans should update the map. Inserting every frame creates excessive redundancy, while aggressive reduction may eliminate useful geometric detail. Keyframe strategies add measurements only after translation, rotation, elapsed time, information gain, or registration-quality thresholds are exceeded. Keyframes therefore provide a compact history of meaningful observations and frequently become nodes in the higher-level optimization structure.

The front end of LiDAR SLAM primarily produces short-term motion estimates and local geometric constraints. Small registration errors nevertheless accumulate over long trajectories and cause drift. A back end addresses this problem using pose graphs, factor graphs, filtering, or nonlinear optimization. Relative LiDAR constraints can be combined with IMU, wheel encoder, GNSS, visual, or other measurements so that the trajectory is estimated from multiple complementary sources rather than from scan matching alone.

Loop closure occurs when the robot recognizes a previously visited region and establishes a constraint between poses that may be widely separated in time. Reliable loop closure can correct accumulated drift across the trajectory and restore global map consistency. Candidate detection may use geometric descriptors, scan context representations, place recognition methods, or map retrieval techniques, followed by precise registration to verify that the two observations genuinely correspond to the same location.

When optimization modifies historical poses, map geometry associated with those poses must remain consistent with the corrected trajectory. Submap-based architectures simplify this process by maintaining locally consistent map fragments connected through optimized transformations. Instead of continuously rebuilding one enormous point cloud, the system can adjust the relative poses of submaps. This architecture supports large-scale mapping while separating high-frequency local registration from lower-frequency global optimization.

Dynamic objects introduce another difficulty because SLAM ideally estimates motion relative to a static environment. Pedestrians, vehicles, forklifts, doors, machinery, vegetation, and other changing objects can produce incorrect correspondences or persistent artifacts in the map. Robust estimators, temporal consistency tests, semantic filtering, occupancy persistence, and correspondence rejection can reduce their influence. Long-term autonomous systems must distinguish transient observations from structures suitable for localization.

Registration quality also depends on environmental observability. A geometrically rich intersection may constrain all pose dimensions strongly, while a long corridor or flat open area can leave particular directions weakly observable. Degeneracy detection examines the geometry or optimization information to identify such conditions. The SLAM system can then increase reliance on inertial, wheel, GNSS, or other measurements instead of accepting an unreliable LiDAR correction with unjustified confidence.

Real-time implementation requires balancing accuracy, map resolution, registration frequency, and computational cost. Point reduction, parallel nearest-neighbor search, voxel hashing, incremental data structures, multiresolution alignment, CPU vectorization, and GPU acceleration can reduce latency. The processing deadline should be related to sensor frequency and vehicle dynamics: an accurate pose estimate delivered too late may be unsuitable for closed-loop autonomous navigation even if its offline error is small.

The complete pipeline therefore operates as a continuous feedback system rather than as an isolated sequence of algorithms. Raw scans are cleaned and deskewed, motion prediction initializes registration, geometric alignment estimates pose correction, validated observations update the local map, and optimized states improve subsequent registration. Loop closure and multi-sensor constraints periodically correct long-term inconsistency, allowing local accuracy and global consistency to reinforce each other throughout operation.

For an industrial or outdoor AMR, robust LiDAR SLAM must ultimately be evaluated as part of the autonomy system rather than only through registration error. Important considerations include localization continuity, recovery after temporary degradation, map stability, computational latency, memory growth, sensitivity to environmental change, and compatibility with navigation. A well-designed scan-registration and map-update pipeline provides the geometric backbone upon which reliable perception, planning, and autonomous operation can be built.

LiDAR SLAM은 연속적인 레이저 스캔(Laser Scan)을 로봇의 움직임과 주변 환경에 대한 일관된 표현으로 변환하는 재귀적 추정 과정(Recursive Estimation Process)이다. 핵심 파이프라인(Pipeline)은 일반적으로 센서 획득(Sensor Acquisition), 전처리(Preprocessing), 모션 보상(Motion Compensation), 스캔 정합(Scan Registration), 자세 추정(Pose Estimation), 지도 갱신(Map Update), 최적화(Optimization)로 구성된다. 각 단계는 로봇 상태와 누적 지도(Map)의 불확실성을 유지하면서 원시 기하 관측(Raw Geometric Observation)을 점차 구조화된 공간 정보로 변환한다.

LiDAR 센서(LiDAR Sensor)는 레이저 에너지(Laser Energy)를 방출하고 주변 표면에서 반사되어 돌아오는 신호를 관측하여 거리를 측정한다. 한 번의 데이터 획득(Acquisition)은 센서 구조에 따라 거리(Range), 각도(Angle), 또는 3차원 점(3D Point)으로 표현되는 스캔(Scan)을 생성한다. 정합(Registration) 이전에는 유효하지 않은 반사값, 지나치게 가깝거나 먼 측정값, 고립된 이상치(Outlier), 중복 포인트(Point)를 제거하는 것이 일반적이다. 복셀 필터링(Voxel Filtering)이나 기하학적 샘플링(Geometric Sampling)을 사용하면 위치 추정에 유용한 구조를 유지하면서 계산량을 줄일 수 있다.

실제 LiDAR 스캔(LiDAR Scan)은 반드시 한순간에 획득되는 측정값은 아니다. 기계식 회전 센서(Mechanical Spinning Sensor)는 일정한 시간 구간 동안 포인트(Point)를 수집하며, 이 과정에서 로봇은 동시에 병진(Translation)하거나 회전(Rotation)할 수 있다. 이는 특히 고속으로 움직이는 야외 자율이동로봇(Outdoor AMR)이나 차량에서 모션 왜곡(Motion Distortion)을 발생시킨다. 디스큐잉(Deskewing)은 타임스탬프(Timestamp)와 IMU, 휠 오도메트리(Wheel Odometry), 이전 LiDAR 상태로부터 추정한 움직임을 이용하여 개별 포인트를 보상하고, 공통 시간 기준으로 정렬된 기하학적으로 일관된 스캔을 생성한다.

정합 단계(Registration Stage)는 현재 스캔(Current Scan)을 이전에 관측된 기하 구조와 가장 잘 정렬하는 강체 변환(Rigid Transformation)을 추정한다. 스캔 대 스캔 정합(Scan-to-Scan Registration)은 연속된 측정값을 비교하고, 스캔 대 지도 정합(Scan-to-Map Registration)은 새로운 관측값을 누적된 로컬 지도(Local Map) 또는 전역 지도(Global Map)에 정렬한다. 추정된 변환은 일반적으로 3차원 병진(Translation)과 회전(Rotation)을 포함하며, SLAM 궤적(Trajectory)을 따라 로봇 자세를 전파하는 데 필요한 상대 운동 제약(Relative Motion Constraint)을 제공한다.

반복 최근접점(Iterative Closest Point, ICP)은 두 포인트 집합(Point Set) 사이의 대응 관계(Correspondence)를 반복적으로 설정하고 기하학적 차이를 최소화하는 대표적인 정합 방법이다. 점 대 점 ICP(Point-to-Point ICP)는 유클리드 거리(Euclidean Distance)를 최소화하며, 점 대 평면(Point-to-Plane) 방식은 로컬 표면 법선(Local Surface Normal)을 활용하여 구조화된 환경에서 보다 효과적으로 수렴할 수 있다. ICP는 개념적으로 단순하지만 초기값(Initialization), 대응점 품질(Correspondence Quality), 환경 기하 구조(Environment Geometry), 동적 객체(Dynamic Object)의 제거 성능에 크게 영향을 받는다.

정규분포 변환(Normal Distributions Transform, NDT)은 명시적인 점 대 점 대응(Point-to-Point Correspondence)을 사용하는 대신 기준 포인트(Reference Point)를 공간 셀(Spatial Cell) 내부의 확률분포(Probability Distribution)로 표현한다. 정합 과정에서는 변환된 스캔 포인트가 이러한 분포와 얼마나 잘 일치하는지를 평가하고 변환을 반복적으로 최적화한다. NDT는 대규모 포인트 클라우드(Point Cloud)의 효율적인 정렬에 활용될 수 있으며 차량 위치 추정에도 널리 사용되어 왔다. 다만 복셀 해상도(Voxel Resolution)와 최적화 파라미터(Optimization Parameter)는 센서 밀도와 환경 규모에 적절하게 설정되어야 한다.

현대적인 LiDAR SLAM 시스템은 정합 전에 기하학적 특징(Geometric Feature)을 추출하는 경우가 많다. 평면 영역(Planar Region), 에지(Edge), 표면 패치(Surface Patch), 또는 국부적으로 구별되는 구조(Local Distinctive Structure)는 모든 포인트를 동일하게 처리하는 것보다 강한 제약 조건을 제공할 수 있다. 특징 선택(Feature Selection)은 데이터 양을 줄이고 안정적인 기하 구조를 강조한다. 그러나 예상되는 구조가 희소한 환경에서는 특징 기반 접근법(Feature-Based Approach)의 성능이 저하될 수 있으므로, 많은 시스템이 특징 추출과 직접 스캔 정합(Direct Scan Matching) 또는 적응형 기하 표현(Adaptive Geometric Representation)을 결합한다.

정합(Registration)은 스캔 사이의 변위가 큰 경우 비선형 최적화(Nonlinear Optimization)가 잘못된 국소해(Local Solution)로 수렴할 수 있기 때문에 충분히 정확한 초기 자세 추정값(Initial Pose Estimate)이 필요하다. 휠 오도메트리(Wheel Odometry), 관성 측정(Inertial Measurement), GNSS, 등속도 모델(Constant-Velocity Model), 이전 SLAM 상태 등이 이러한 예측값을 제공할 수 있다. 이후 LiDAR 정합이 누적된 운동 오차를 보정한다. 이러한 예측-보정(Prediction-Correction) 관계는 고속 로봇, 거친 지형(Rough Terrain), 급격한 회전, 기하학적 제약이 부족한 환경에서 특히 중요하다.

정합 이후 추정된 상대 변환(Relative Transformation)은 로봇 궤적(Robot Trajectory)에 통합된다. 현재 자세(Current Pose)는 이전 자세, 예측된 움직임(Predicted Motion), 정합 보정값(Registration Correction)의 합성으로 표현할 수 있다. 실제 시스템에서는 잔차 오차(Residual Error), 대응점 통계(Correspondence Statistics), 기하학적 퇴화(Geometric Degeneracy), 공분산 근사(Covariance Approximation) 등을 이용하여 정합 신뢰도(Registration Confidence)를 추정하기도 한다. 복도, 터널, 넓은 평면과 같이 특정 방향의 제약이 약한 환경에서는 겉보기에는 성공적인 정합이라도 해당 방향에 상당한 불확실성이 존재할 수 있으므로 이를 인식해야 한다.

지도 갱신(Map Update)은 새롭게 정합된 측정값을 공간 표현(Spatial Representation)의 일부로 만드는 방법을 결정한다. 단순한 포인트 클라우드 지도(Point-Cloud Map)는 변환된 포인트를 전역 좌표계(Global Coordinate Frame)에 직접 삽입할 수 있지만, 무제한적인 누적은 메모리와 계산량을 빠르게 증가시킨다. 따라서 복셀 그리드(Voxel Grid), 옥트리(Octree), 서펠(Surfel), 확률분포(Probability Distribution), 계층형 서브맵(Hierarchical Submap) 등이 보다 확장 가능한 표현으로 활용된다. 지도 표현 방식은 지도가 주로 위치 추정(Localization), 내비게이션(Navigation), 장애물 판단(Obstacle Reasoning), 시각화(Visualization), 장기 운용(Long-Term Operation) 중 어떤 목적으로 사용되는지에 따라 달라진다.

많은 실시간 시스템(Real-Time System)은 현재 로봇 위치 주변의 기하 구조만 포함하는 로컬 지도(Local Map)를 유지한다. 입력되는 스캔은 이 로컬 표현(Local Representation)에 대해 정합되며, 로봇이 이동하면 오래된 영역은 제거되거나 더 큰 서브맵(Submap)으로 이전된다. 이를 통해 계산 복잡도(Computational Complexity)를 제한하고 정합 과정이 현재 필요한 기하 구조에 집중하도록 할 수 있다. 로컬 매핑(Local Mapping)은 대규모 시설, 도로, 산업 현장, 항만, 캠퍼스, 물류 환경에서 운용되는 야외 자율이동로봇(Outdoor AMR)에 특히 효과적이다.

실제 시스템 설계에서 중요한 문제 중 하나는 어떤 정합 스캔(Registered Scan)을 지도 갱신에 사용할 것인지 결정하는 것이다. 모든 프레임(Frame)을 삽입하면 과도한 중복이 발생하지만 지나치게 공격적으로 데이터를 줄이면 유용한 기하학적 세부 정보가 손실될 수 있다. 키프레임 전략(Keyframe Strategy)은 병진량, 회전량, 경과 시간, 정보 이득(Information Gain), 정합 품질(Registration Quality) 등의 임계값이 초과될 때만 측정값을 추가한다. 따라서 키프레임(Keyframe)은 의미 있는 관측의 압축된 이력을 제공하며 상위 수준 최적화 구조(Optimization Structure)의 노드(Node)로 사용되는 경우가 많다.

LiDAR SLAM의 프론트엔드(Front End)는 주로 단기 운동 추정(Short-Term Motion Estimation)과 로컬 기하 제약(Local Geometric Constraint)을 생성한다. 그러나 작은 정합 오차도 긴 궤적에서는 누적되어 드리프트(Drift)를 발생시킨다. 백엔드(Back End)는 포즈 그래프(Pose Graph), 팩터 그래프(Factor Graph), 필터링(Filtering), 비선형 최적화(Nonlinear Optimization)를 사용하여 이러한 문제를 해결한다. 상대 LiDAR 제약은 IMU, 휠 인코더(Wheel Encoder), GNSS, 비전(Visual) 또는 다른 센서 측정값과 결합될 수 있으며, 이를 통해 스캔 정합만 사용하는 대신 여러 상호보완적인 정보원으로부터 궤적을 추정할 수 있다.

루프 폐쇄(Loop Closure)는 로봇이 이전에 방문했던 영역을 다시 인식하고 시간적으로 멀리 떨어진 두 자세 사이에 제약 조건을 설정할 때 발생한다. 신뢰성 높은 루프 폐쇄는 전체 궤적에서 누적된 드리프트를 보정하고 전역 지도의 일관성(Global Map Consistency)을 복원할 수 있다. 후보 영역 검출(Candidate Detection)에는 기하학적 디스크립터(Geometric Descriptor), 스캔 컨텍스트 표현(Scan Context Representation), 장소 인식(Place Recognition), 지도 검색(Map Retrieval) 기법 등을 사용할 수 있으며, 이후 정밀 정합(Precise Registration)을 수행하여 두 관측값이 실제로 동일한 위치에 대응하는지 검증한다.

최적화(Optimization)가 과거 자세(Historical Pose)를 수정하면 해당 자세와 연결된 지도 기하 구조(Map Geometry) 역시 보정된 궤적과 일관성을 유지해야 한다. 서브맵 기반 아키텍처(Submap-Based Architecture)는 최적화된 변환으로 연결된 국부적으로 일관된 지도 조각을 유지하여 이러한 과정을 단순화한다. 하나의 거대한 포인트 클라우드를 지속적으로 다시 구성하는 대신 서브맵 사이의 상대 자세(Relative Pose)를 조정할 수 있다. 이러한 구조는 고주파 로컬 정합(High-Frequency Local Registration)과 저주파 전역 최적화(Lower-Frequency Global Optimization)를 분리하면서 대규모 매핑(Large-Scale Mapping)을 지원한다.

동적 객체(Dynamic Object)는 SLAM이 이상적으로 정적 환경(Static Environment)을 기준으로 움직임을 추정한다는 점에서 또 다른 어려움을 발생시킨다. 보행자, 차량, 지게차, 문, 기계, 식생 등 변화하는 객체는 잘못된 대응 관계를 생성하거나 지도에 지속적인 인공 흔적(Artifact)을 남길 수 있다. 강인 추정기(Robust Estimator), 시간적 일관성 검사(Temporal Consistency Test), 의미론적 필터링(Semantic Filtering), 점유 지속성(Occupancy Persistence), 대응점 제거(Correspondence Rejection)를 통해 이러한 영향을 줄일 수 있다. 장기 자율 시스템(Long-Term Autonomous System)은 일시적인 관측과 위치 추정에 적합한 지속적인 구조를 구분해야 한다.

정합 품질(Registration Quality)은 환경의 관측 가능성(Environmental Observability)에도 영향을 받는다. 기하학적 구조가 풍부한 교차 구역은 모든 자세 차원(Pose Dimension)을 강하게 제약할 수 있지만, 긴 복도나 평평한 개방 공간에서는 특정 방향의 관측 가능성이 낮아질 수 있다. 퇴화 검출(Degeneracy Detection)은 기하 구조나 최적화 정보를 분석하여 이러한 상태를 식별한다. SLAM 시스템은 신뢰하기 어려운 LiDAR 보정값을 과도하게 수용하는 대신 IMU, 휠, GNSS 또는 다른 센서 측정값에 대한 의존도를 증가시킬 수 있다.

실시간 구현(Real-Time Implementation)에서는 정확도, 지도 해상도(Map Resolution), 정합 주기(Registration Frequency), 계산 비용(Computational Cost) 사이의 균형이 필요하다. 포인트 감소(Point Reduction), 병렬 최근접점 검색(Parallel Nearest-Neighbor Search), 복셀 해싱(Voxel Hashing), 증분 데이터 구조(Incremental Data Structure), 다중 해상도 정합(Multiresolution Alignment), CPU 벡터화(CPU Vectorization), GPU 가속(GPU Acceleration) 등을 이용하여 지연 시간(Latency)을 줄일 수 있다. 처리 마감 시간(Processing Deadline)은 센서 주기와 차량 동역학(Vehicle Dynamics)을 고려하여 설정해야 하며, 정확한 자세 추정이라도 지나치게 늦게 제공된다면 폐루프 자율주행(Closed-Loop Autonomous Navigation)에 적합하지 않을 수 있다.

따라서 전체 파이프라인은 독립된 알고리즘의 단순한 순차 실행이 아니라 지속적인 피드백 시스템(Feedback System)으로 동작한다. 원시 스캔(Raw Scan)은 정제 및 디스큐잉되고, 운동 예측(Motion Prediction)은 정합 초기값을 제공하며, 기하 정렬(Geometric Alignment)은 자세 보정값을 추정한다. 검증된 관측값은 로컬 지도를 갱신하고 최적화된 상태는 이후의 정합 성능을 향상시킨다. 루프 폐쇄와 다중 센서 제약(Multi-Sensor Constraint)은 장기적인 불일치를 주기적으로 보정하여 로컬 정확도(Local Accuracy)와 전역 일관성(Global Consistency)이 전체 운용 과정에서 상호 강화되도록 한다.

산업용 또는 야외 자율이동로봇(Industrial or Outdoor AMR)의 경우 강인한 LiDAR SLAM은 단순히 정합 오차(Registration Error)만으로 평가하는 것이 아니라 전체 자율주행 시스템(Autonomy System)의 일부로 평가해야 한다. 위치 추정 연속성(Localization Continuity), 일시적 성능 저하 이후의 복구(Recovery), 지도 안정성(Map Stability), 계산 지연(Computational Latency), 메모리 증가(Memory Growth), 환경 변화에 대한 민감도, 내비게이션과의 호환성 등이 중요한 평가 요소가 된다. 잘 설계된 스캔 정합 및 지도 갱신 파이프라인은 신뢰할 수 있는 인지(Perception), 경로 계획(Planning), 자율 운용(Autonomous Operation)을 구축하기 위한 핵심적인 기하학적 기반(Geometric Backbone)을 제공한다.

##  

## 06.02. ICP Iterative Closest Point Variants [w/Code]

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

Iterative Closest Point, or ICP, is one of the fundamental geometric registration methods used in LiDAR SLAM, localization, 3D reconstruction, and robotic perception. Its objective is to estimate the rigid transformation that aligns a source point cloud with a target point cloud or map. ICP repeatedly establishes correspondences, computes an alignment error, estimates a transformation, and updates the source until a convergence condition is reached.

The classical ICP pipeline begins with an initial estimate of the relative pose between two point sets. Each source point is associated with its nearest target point according to a distance metric, usually Euclidean distance. The transformation that minimizes the resulting correspondence error is then calculated and applied to the source cloud. These steps are repeated until the pose increment, residual error, or maximum iteration criterion indicates convergence.

A central limitation of ICP is that it performs local optimization rather than global registration. If the initial transformation is sufficiently close to the correct alignment, the algorithm can converge accurately and efficiently. If initialization is poor, however, nearest-neighbor associations may represent unrelated surfaces and guide optimization toward an incorrect local minimum. LiDAR odometry therefore commonly supplies ICP with predictions from previous poses, IMUs, wheel odometry, or motion models.

Point-to-point ICP is the most direct formulation. It minimizes the sum of squared Euclidean distances between corresponding source and target points. The optimal rigid transformation can be calculated using methods based on singular value decomposition or related closed-form solutions. Point-to-point ICP is simple and broadly applicable, but it may converge slowly because it ignores the local orientation and surface structure represented by neighboring points.

Point-to-plane ICP improves registration by minimizing the distance from each transformed source point to the tangent plane associated with its corresponding target point. Surface normals are estimated from local neighborhoods in the target cloud and incorporated into the residual function. This formulation captures local geometry more effectively than point-to-point distance and often provides faster convergence in environments dominated by walls, floors, roads, and other smooth surfaces.

Generalized ICP, commonly called GICP, combines concepts from point-to-point and point-to-plane registration using local covariance models. Each point is associated with a covariance matrix describing the geometric distribution of its neighborhood. Registration minimizes a probabilistically weighted error between corresponding local structures. This allows planar, linear, and volumetric geometry to influence optimization differently and often improves robustness for three-dimensional LiDAR data.

Plane-to-plane and distribution-aware variants extend this principle by aligning local surface models rather than individual measurements. Groups of neighboring points may be represented by planes, surfels, Gaussian distributions, or covariance ellipsoids. Such representations suppress measurement-level noise and provide compact geometric constraints. They are especially useful when high-density LiDAR produces many redundant samples from the same physical surfaces.

Correspondence generation strongly affects every ICP variant. A basic nearest-neighbor search may associate geometrically close but physically unrelated points, particularly around edges, repeated structures, or dynamic objects. Distance thresholds, normal-angle compatibility, reciprocal correspondences, neighborhood consistency, and semantic constraints can reject implausible matches. Efficient spatial structures such as k-d trees and voxel indexes reduce the cost of repeatedly searching large target clouds.

Outlier rejection is equally important because real-world point clouds rarely contain perfect overlap. Parts of the source scan may observe areas absent from the target, while moving vehicles, pedestrians, vegetation, reflective surfaces, and sensor noise create inconsistent measurements. Trimmed ICP retains only a selected fraction of low-residual correspondences, while robust kernels reduce the influence of large residuals instead of treating every match equally during optimization.

Robust ICP formulations frequently use M-estimators such as Huber, Cauchy, or Tukey functions to modify the contribution of residuals. Small errors remain influential because they are likely to represent valid correspondences, whereas unusually large errors receive reduced weight. Robust loss functions do not eliminate incorrect associations automatically, but when combined with correspondence filtering they significantly improve registration stability in cluttered and partially dynamic environments.

Weighted ICP assigns different importance to correspondences according to measurement reliability or geometric information. Weights may depend on range uncertainty, incidence angle, surface normal quality, local curvature, semantic class, or estimated sensor noise. For example, a stable building facade may provide stronger localization information than sparse vegetation. Weighting enables the optimizer to emphasize measurements that contribute reliable constraints to the desired motion dimensions.

Colored ICP combines geometric residuals with photometric or color information when point clouds contain RGB measurements. Geometry constrains spatial alignment, while appearance helps distinguish regions that are geometrically similar but visually different. Although this method is particularly relevant to RGB-D cameras and colored point clouds, the broader concept illustrates how ICP can incorporate additional observation channels rather than relying exclusively on Euclidean geometry.

Multi-resolution ICP addresses large point clouds and relatively large initial pose errors by performing registration across several spatial scales. Coarsely downsampled clouds are aligned first, providing a stable approximate transformation with relatively low computational cost. Finer-resolution levels then refine the solution using increasingly detailed geometry. This coarse-to-fine strategy enlarges the practical convergence region while reducing unnecessary processing of dense data during early iterations.

Voxelized ICP variants organize points into discrete spatial cells and perform correspondence or optimization operations using voxel-level structures. Voxelization can reduce memory access, accelerate neighborhood queries, and provide natural multiresolution representations. Modern implementations may combine voxel hashing, parallel correspondence search, SIMD processing, or GPU acceleration. These techniques are important when high-frequency 3D LiDAR must support real-time localization on autonomous mobile platforms.

Sparse ICP introduces sparsity-promoting formulations that can reduce sensitivity to large correspondence errors. Rather than relying purely on a conventional squared-error objective, sparse penalties encourage the optimizer to tolerate a limited number of substantial residuals while maintaining accurate alignment for consistent measurements. Such approaches can improve robustness under partial overlap and outliers, although their optimization procedures may be more computationally complex than classical ICP.

Probabilistic ICP variants explicitly model uncertainty in measurements, correspondences, or transformations. Instead of assuming that every point represents an exact location, these methods describe observations through probability distributions and optimize an uncertainty-aware objective. Covariance information can originate from sensor characteristics, local geometry, or state estimation. This framework provides a natural connection between scan registration and probabilistic SLAM back ends.

ICP can also be categorized according to the registration target. Scan-to-scan ICP aligns the current scan with the immediately preceding scan and therefore requires relatively limited map storage, but incremental errors can accumulate rapidly. Scan-to-map ICP aligns observations against an accumulated local map containing geometry from multiple previous scans. The richer target generally improves stability and reduces short-term drift, although map maintenance and computational complexity increase.

Submap-based ICP provides a compromise between purely consecutive registration and alignment against a continuously expanding global map. The current scan is matched against a bounded local submap constructed from selected keyframes. As the robot moves, the active submap is updated or replaced while completed submaps become higher-level mapping entities. This architecture supports predictable computation and integrates naturally with pose-graph optimization and loop closure.

Degenerate environments reveal an important limitation shared by ICP variants. A long planar wall can strongly constrain motion perpendicular to the surface while providing little information about motion parallel to it. Corridors, tunnels, flat roads, and open areas may similarly leave certain degrees of freedom weakly observable. Eigenvalue analysis, Hessian conditioning, or covariance estimation can identify these directions so that unreliable ICP corrections are not assigned excessive confidence.

Stopping criteria determine when iterative refinement should terminate. Registration may stop when translation and rotation updates fall below predefined thresholds, when the change in objective value becomes sufficiently small, or when a maximum number of iterations is reached. Practical real-time systems must balance numerical convergence against latency. Continuing optimization after meaningful improvement has stopped wastes computational resources without necessarily producing a more useful pose estimate.

The selection of an ICP variant should therefore reflect sensor characteristics, environment geometry, computational resources, and expected motion rather than treating one formulation as universally optimal. Point-to-point ICP offers simplicity, point-to-plane ICP efficiently exploits surfaces, GICP incorporates local geometric uncertainty, and robust or weighted variants address difficult measurements. Multiresolution and voxelized implementations further improve convergence behavior and real-time performance.

Within LiDAR SLAM, ICP is best understood as a configurable registration framework rather than a single fixed algorithm. Reliable systems combine appropriate residual models, correspondence policies, outlier rejection, initialization, uncertainty estimation, and computational acceleration. When these components are integrated with IMU, wheel odometry, GNSS, local mapping, and global optimization, ICP variants provide precise geometric constraints that form a central part of robust robot localization and mapping.

반복 최근접점(Iterative Closest Point, ICP)은 LiDAR SLAM, 위치 추정(Localization), 3차원 재구성(3D Reconstruction), 로봇 인지(Robotic Perception)에서 사용되는 대표적인 기하학적 정합(Geometric Registration) 방법 중 하나이다. ICP의 목적은 소스 포인트 클라우드(Source Point Cloud)를 대상 포인트 클라우드(Target Point Cloud) 또는 지도(Map)에 정렬하는 강체 변환(Rigid Transformation)을 추정하는 것이다. ICP는 대응점(Correspondence)을 설정하고, 정렬 오차(Alignment Error)를 계산하며, 변환을 추정한 뒤 소스를 갱신하는 과정을 수렴 조건(Convergence Condition)이 만족될 때까지 반복한다.

고전적인 ICP 파이프라인(ICP Pipeline)은 두 포인트 집합(Point Set) 사이의 상대 자세(Relative Pose)에 대한 초기 추정값에서 시작한다. 각 소스 포인트(Source Point)는 일반적으로 유클리드 거리(Euclidean Distance)를 사용하는 거리 척도(Distance Metric)에 따라 가장 가까운 대상 포인트(Target Point)와 연결된다. 이후 대응점 오차를 최소화하는 변환을 계산하여 소스 클라우드에 적용한다. 이러한 과정은 자세 변화량(Pose Increment), 잔차 오차(Residual Error), 최대 반복 횟수(Maximum Iteration) 등의 조건이 수렴을 나타낼 때까지 반복된다.

ICP의 핵심적인 한계는 전역 정합(Global Registration)이 아니라 국소 최적화(Local Optimization)를 수행한다는 점이다. 초기 변환(Initial Transformation)이 올바른 정렬에 충분히 가까우면 알고리즘은 정확하고 효율적으로 수렴할 수 있다. 그러나 초기값이 부정확하면 최근접점 대응 관계가 서로 관련 없는 표면을 연결하여 잘못된 국소 최솟값(Local Minimum)으로 최적화를 유도할 수 있다. 따라서 LiDAR 오도메트리(LiDAR Odometry)는 이전 자세, IMU, 휠 오도메트리(Wheel Odometry), 운동 모델(Motion Model) 등으로부터 얻은 예측값을 ICP 초기값으로 사용하는 경우가 많다.

점 대 점 ICP(Point-to-Point ICP)는 가장 직접적인 형태의 ICP이다. 대응하는 소스 포인트와 대상 포인트 사이의 제곱 유클리드 거리(Squared Euclidean Distance)의 합을 최소화한다. 최적의 강체 변환은 특이값 분해(Singular Value Decomposition, SVD) 또는 관련된 폐쇄형 해법(Closed-Form Solution)을 이용하여 계산할 수 있다. 점 대 점 ICP는 단순하고 폭넓게 적용할 수 있지만 인접 포인트가 표현하는 국부 방향(Local Orientation)과 표면 구조(Surface Structure)를 활용하지 않기 때문에 수렴 속도가 느려질 수 있다.

점 대 평면 ICP(Point-to-Plane ICP)는 변환된 각 소스 포인트와 이에 대응하는 대상 포인트의 접평면(Tangent Plane) 사이 거리를 최소화하여 정합 성능을 향상시킨다. 표면 법선(Surface Normal)은 대상 클라우드의 국부 이웃(Local Neighborhood)으로부터 추정되어 잔차 함수(Residual Function)에 포함된다. 이 방식은 점 대 점 거리보다 국부 기하 구조(Local Geometry)를 효과적으로 활용하며 벽, 바닥, 도로 등 매끄러운 표면이 많은 환경에서 더 빠른 수렴 성능을 제공하는 경우가 많다.

일반화 ICP(Generalized ICP, GICP)는 국부 공분산 모델(Local Covariance Model)을 사용하여 점 대 점 정합과 점 대 평면 정합의 개념을 결합한다. 각 포인트에는 주변 기하 분포(Geometric Distribution)를 표현하는 공분산 행렬(Covariance Matrix)이 연결된다. 정합 과정에서는 서로 대응하는 국부 구조 사이의 확률적으로 가중된 오차(Probabilistically Weighted Error)를 최소화한다. 이를 통해 평면형(Planar), 선형(Linear), 체적형(Volumetric) 기하 구조가 최적화에 서로 다르게 영향을 미칠 수 있으며 3차원 LiDAR 데이터에서 강인성(Robustness)을 향상시킬 수 있다.

평면 대 평면(Plane-to-Plane) 및 분포 인식형(Distribution-Aware) 변형은 개별 측정값 대신 국부 표면 모델(Local Surface Model)을 정렬함으로써 이러한 개념을 확장한다. 인접한 포인트 그룹은 평면(Plane), 서펠(Surfel), 가우시안 분포(Gaussian Distribution), 공분산 타원체(Covariance Ellipsoid) 등으로 표현할 수 있다. 이러한 표현은 측정 수준의 노이즈를 억제하면서 압축된 기하 제약(Geometric Constraint)을 제공하며, 고밀도 LiDAR가 동일한 물리적 표면에서 많은 중복 샘플을 생성하는 경우 특히 유용하다.

대응점 생성(Correspondence Generation)은 모든 ICP 변형의 성능에 큰 영향을 미친다. 기본적인 최근접 이웃 탐색(Nearest-Neighbor Search)은 특히 에지(Edge), 반복 구조(Repeated Structure), 동적 객체(Dynamic Object) 주변에서 기하학적으로 가깝지만 실제로는 관계없는 포인트를 연결할 수 있다. 거리 임계값(Distance Threshold), 법선 각도 호환성(Normal-Angle Compatibility), 상호 대응점(Reciprocal Correspondence), 이웃 일관성(Neighborhood Consistency), 의미론적 제약(Semantic Constraint)을 사용하여 잘못된 대응을 제거할 수 있다. k-d 트리(k-d Tree)와 복셀 인덱스(Voxel Index) 같은 효율적인 공간 구조는 대규모 대상 클라우드에서 반복되는 탐색 비용을 줄인다.

이상치 제거(Outlier Rejection) 역시 중요하다. 실제 포인트 클라우드는 완벽하게 중첩되는 경우가 거의 없기 때문이다. 소스 스캔의 일부 영역은 대상에 존재하지 않을 수 있으며, 이동 차량, 보행자, 식생, 반사 표면, 센서 노이즈 등이 일관되지 않은 측정값을 생성한다. 절삭 ICP(Trimmed ICP)는 잔차가 작은 일정 비율의 대응점만 유지하며, 강인 커널(Robust Kernel)은 최적화 과정에서 모든 대응점을 동일하게 취급하는 대신 큰 잔차의 영향을 감소시킨다.

강인 ICP(Robust ICP)는 잔차의 기여도를 조절하기 위해 후버(Huber), 코시(Cauchy), 튜키(Tukey) 함수와 같은 M-추정기(M-Estimator)를 자주 사용한다. 작은 오차는 유효한 대응점일 가능성이 높기 때문에 충분한 영향력을 유지하지만 비정상적으로 큰 오차에는 낮은 가중치를 부여한다. 강인 손실 함수(Robust Loss Function)가 잘못된 대응점을 자동으로 제거하는 것은 아니지만 대응점 필터링(Correspondence Filtering)과 결합하면 복잡하고 부분적으로 동적인 환경에서 정합 안정성을 크게 향상시킬 수 있다.

가중 ICP(Weighted ICP)는 측정 신뢰도(Measurement Reliability) 또는 기하학적 정보량에 따라 각 대응점에 서로 다른 중요도를 부여한다. 가중치는 거리 불확실성(Range Uncertainty), 입사각(Incidence Angle), 표면 법선 품질(Surface Normal Quality), 국부 곡률(Local Curvature), 의미론적 클래스(Semantic Class), 추정된 센서 노이즈 등에 따라 결정할 수 있다. 예를 들어 안정적인 건물 외벽(Building Facade)은 희소한 식생보다 신뢰성 높은 위치 추정 정보를 제공할 수 있다. 이러한 가중화는 원하는 운동 차원(Motion Dimension)에 신뢰성 높은 제약을 제공하는 측정값을 최적화 과정에서 강조할 수 있도록 한다.

컬러 ICP(Colored ICP)는 포인트 클라우드가 RGB 측정값을 포함하는 경우 기하학적 잔차(Geometric Residual)와 광도 또는 색상 정보(Photometric or Color Information)를 결합한다. 기하 정보는 공간 정렬(Spatial Alignment)을 제약하고 외관 정보(Appearance Information)는 기하학적으로 유사하지만 시각적으로 다른 영역을 구별하는 데 도움을 준다. 이 방법은 RGB-D 카메라와 컬러 포인트 클라우드에 특히 관련성이 높지만, ICP가 유클리드 기하 정보만 사용하는 대신 추가적인 관측 채널(Observation Channel)을 통합할 수 있다는 일반적인 개념을 보여준다.

다중 해상도 ICP(Multi-Resolution ICP)는 여러 공간 해상도(Spatial Scale)에서 정합을 수행하여 대규모 포인트 클라우드와 비교적 큰 초기 자세 오차를 처리한다. 먼저 거칠게 다운샘플링된 클라우드(Coarsely Downsampled Cloud)를 정렬하여 비교적 낮은 계산 비용으로 안정적인 근사 변환을 얻는다. 이후 더 높은 해상도 단계에서 점차 세밀한 기하 정보를 이용하여 해를 정제한다. 이러한 거친 단계에서 세밀한 단계로 진행하는 전략(Coarse-to-Fine Strategy)은 실질적인 수렴 영역(Convergence Region)을 확장하면서 초기 반복 과정에서 고밀도 데이터를 불필요하게 처리하는 것을 줄인다.

복셀화 ICP(Voxelized ICP) 변형은 포인트를 이산적인 공간 셀(Spatial Cell)로 구성하고 복셀 수준(Voxel Level)의 구조를 이용하여 대응점 탐색이나 최적화를 수행한다. 복셀화(Voxelization)는 메모리 접근을 줄이고 이웃 탐색을 가속하며 자연스러운 다중 해상도 표현(Multiresolution Representation)을 제공할 수 있다. 현대적인 구현에서는 복셀 해싱(Voxel Hashing), 병렬 대응점 탐색(Parallel Correspondence Search), SIMD 처리, GPU 가속(GPU Acceleration)을 결합하기도 한다. 이러한 기법은 고주파 3차원 LiDAR를 자율이동 플랫폼의 실시간 위치 추정에 사용하는 경우 중요하다.

희소 ICP(Sparse ICP)는 큰 대응점 오차에 대한 민감도를 줄이기 위해 희소성을 촉진하는 목적 함수(Sparsity-Promoting Formulation)를 도입한다. 기존의 제곱 오차 목적 함수에만 의존하는 대신 희소 패널티(Sparse Penalty)를 사용하여 제한된 수의 큰 잔차를 허용하면서 일관된 측정값에 대해서는 정확한 정렬을 유지하도록 한다. 이러한 방식은 부분 중첩(Partial Overlap)과 이상치가 존재하는 환경에서 강인성을 높일 수 있지만 최적화 과정은 고전적인 ICP보다 계산적으로 복잡해질 수 있다.

확률적 ICP(Probabilistic ICP) 변형은 측정값, 대응 관계 또는 변환의 불확실성(Uncertainty)을 명시적으로 모델링한다. 모든 포인트가 정확한 위치를 나타낸다고 가정하는 대신 관측값을 확률분포(Probability Distribution)로 표현하고 불확실성을 고려한 목적 함수(Uncertainty-Aware Objective)를 최적화한다. 공분산 정보(Covariance Information)는 센서 특성, 국부 기하 구조 또는 상태 추정(State Estimation)으로부터 얻을 수 있다. 이러한 구조는 스캔 정합과 확률적 SLAM(Probabilistic SLAM) 백엔드 사이에 자연스러운 연결을 제공한다.

ICP는 정합 대상(Registration Target)에 따라서도 구분할 수 있다. 스캔 대 스캔 ICP(Scan-to-Scan ICP)는 현재 스캔을 바로 이전 스캔과 정렬하기 때문에 상대적으로 적은 지도 저장 공간이 필요하지만 증분 오차(Incremental Error)가 빠르게 누적될 수 있다. 스캔 대 지도 ICP(Scan-to-Map ICP)는 현재 관측값을 여러 이전 스캔으로 구성된 누적 로컬 지도(Local Map)에 정렬한다. 더 풍부한 대상 기하 구조를 이용하므로 일반적으로 안정성이 향상되고 단기 드리프트(Short-Term Drift)를 줄일 수 있지만 지도 유지와 계산 복잡도가 증가한다.

서브맵 기반 ICP(Submap-Based ICP)는 연속된 스캔만을 이용하는 정합과 지속적으로 확장되는 전역 지도(Global Map)를 이용하는 정합 사이의 절충안을 제공한다. 현재 스캔은 선택된 키프레임(Keyframe)으로 구성된 제한된 크기의 로컬 서브맵(Local Submap)에 정합된다. 로봇이 이동하면 활성 서브맵(Active Submap)이 갱신되거나 교체되고 완료된 서브맵은 상위 수준의 매핑 요소(Mapping Entity)가 된다. 이러한 아키텍처는 예측 가능한 계산량을 제공하면서 포즈 그래프 최적화(Pose-Graph Optimization) 및 루프 폐쇄(Loop Closure)와 자연스럽게 통합된다.

퇴화 환경(Degenerate Environment)은 ICP 변형들이 공통적으로 갖는 중요한 한계를 보여준다. 긴 평면 벽은 표면에 수직인 방향의 움직임을 강하게 제약하지만 표면과 평행한 방향의 움직임에 대해서는 거의 정보를 제공하지 못할 수 있다. 복도, 터널, 평탄한 도로, 개방 공간에서도 특정 자유도(Degree of Freedom)의 관측 가능성(Observability)이 낮아질 수 있다. 고유값 분석(Eigenvalue Analysis), 헤시안 조건 분석(Hessian Conditioning), 공분산 추정(Covariance Estimation)을 이용하여 이러한 방향을 식별하면 신뢰성이 낮은 ICP 보정값에 과도한 신뢰도를 부여하는 것을 방지할 수 있다.

종료 조건(Stopping Criteria)은 반복적인 정제 과정(Iterative Refinement)을 언제 끝낼 것인지를 결정한다. 병진과 회전 업데이트가 사전에 정의된 임계값 이하로 감소하거나 목적 함수 값(Objective Value)의 변화가 충분히 작아지거나 최대 반복 횟수에 도달하면 정합을 종료할 수 있다. 실제 실시간 시스템은 수치적 수렴(Numerical Convergence)과 지연 시간(Latency) 사이의 균형을 유지해야 한다. 의미 있는 개선이 중단된 이후에도 최적화를 계속하면 더 유용한 자세 추정값을 얻지 못하면서 계산 자원만 소비할 수 있다.

따라서 ICP 변형의 선택은 하나의 방법을 보편적인 최적 해법으로 간주하기보다 센서 특성(Sensor Characteristics), 환경 기하 구조(Environment Geometry), 계산 자원(Computational Resources), 예상 운동(Expected Motion)을 고려하여 결정해야 한다. 점 대 점 ICP는 단순성을 제공하고, 점 대 평면 ICP는 표면 구조를 효율적으로 활용하며, GICP는 국부 기하 불확실성을 통합한다. 강인 ICP와 가중 ICP는 어려운 측정 환경을 처리하며, 다중 해상도 및 복셀화 구현은 수렴 특성과 실시간 성능을 추가적으로 향상시킨다.

LiDAR SLAM에서 ICP는 하나의 고정된 알고리즘이라기보다 구성 가능한 정합 프레임워크(Configurable Registration Framework)로 이해하는 것이 적절하다. 신뢰성 높은 시스템은 적절한 잔차 모델(Residual Model), 대응점 정책(Correspondence Policy), 이상치 제거, 초기화(Initialization), 불확실성 추정(Uncertainty Estimation), 계산 가속(Computational Acceleration)을 결합한다. 이러한 구성 요소를 IMU, 휠 오도메트리, GNSS, 로컬 매핑(Local Mapping), 전역 최적화(Global Optimization)와 통합하면 다양한 ICP 변형은 강인한 로봇 위치 추정과 지도 작성의 핵심을 이루는 정밀한 기하학적 제약(Geometric Constraint)을 제공할 수 있다.

##  

## 06.03. NDT Normal Distributions Transform SLAM [w/Code]

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

Normal Distributions Transform, or NDT, is a geometric registration method widely used for LiDAR localization and SLAM. Instead of directly matching individual points between two scans, NDT converts the reference point cloud into a continuous probabilistic representation of local geometry. Registration then estimates the rigid transformation that places the incoming scan where its points achieve high likelihood within these spatial distributions.

The fundamental idea of NDT is to divide three-dimensional space into a set of cells or voxels. Points contained within each occupied voxel are summarized statistically rather than stored only as independent measurements. A mean vector describes the local center of the observations, while a covariance matrix characterizes their spatial distribution. Each populated cell can therefore be approximated as a multivariate Gaussian distribution representing local environmental geometry.

This statistical representation distinguishes NDT from correspondence-driven methods such as classical ICP. ICP repeatedly searches for explicit point-to-point or point-to-surface correspondences, whereas NDT evaluates transformed scan points against probability distributions defined by the reference map. The optimization objective is therefore based on distribution likelihood rather than a collection of discrete nearest-neighbor distances, reducing dependence on explicit correspondence searches during iterative registration.

The covariance matrix inside each NDT cell contains useful geometric information. Points sampled from a planar wall produce a distribution with large variance along the surface and small variance perpendicular to it. Linear structures produce another characteristic eigenvalue pattern, while irregular volumetric regions generate more balanced distributions. NDT consequently captures local orientation and shape implicitly through statistics without requiring every raw point to participate independently in optimization.

The registration process begins with an initial estimate of the sensor or robot pose. Each point from the current LiDAR scan is transformed according to this estimate and associated with the corresponding NDT cell in the reference representation. The probability density of the transformed point is evaluated using the cell mean and covariance. Contributions from many scan points are combined into an objective function that measures overall agreement between the scan and map.

Optimization adjusts the translation and rotation parameters to maximize the likelihood, or equivalently minimize an appropriate negative score, of the transformed scan within the NDT map. Gradients and, in many implementations, second-order information are calculated from the probabilistic objective. Newton-type, quasi-Newton, or related numerical methods can then iteratively update the pose until the transformation converges or predefined termination conditions are reached.

As with ICP, NDT is fundamentally a local registration technique and therefore benefits from a reliable initial pose estimate. Previous SLAM poses, wheel odometry, IMU integration, GNSS, or constant-velocity prediction can place the optimizer within an appropriate convergence region. A poor initialization may cause the transformed points to enter unrelated distributions, resulting in convergence toward an incorrect pose or failure to obtain a stable solution.

Voxel resolution is one of the most influential NDT parameters. Large cells contain many points and produce smooth distributions that can tolerate relatively large initial alignment errors, but they may suppress fine environmental details. Small cells preserve more geometric structure and potentially support precise registration, yet they may contain too few measurements for reliable covariance estimation and create a more complex optimization landscape.

Multi-resolution NDT addresses this tradeoff by performing registration at several voxel scales. Coarse distributions are used first to estimate large-scale alignment and provide a stable pose approximation. Registration is then repeated using progressively finer cells that preserve more detailed geometry. This coarse-to-fine strategy can enlarge the effective convergence region while retaining the accuracy available from high-resolution spatial information near the final solution.

NDT can operate in two-dimensional or three-dimensional form depending on the sensor and application. 2D NDT represents laser measurements in planar cells and is suitable for ground robots operating in approximately planar environments. 3D NDT models volumetric space using three-dimensional Gaussian distributions and is more appropriate for modern multi-beam LiDAR systems, autonomous vehicles, outdoor AMRs, mining machines, and robots moving through complex terrain.

Scan-to-scan NDT aligns the current LiDAR measurement with a representation constructed from a previous scan. This architecture is relatively simple and limits the amount of reference data, but errors can accumulate because every estimate depends strongly on preceding registrations. Scan-to-map NDT instead aligns the current scan against a local map containing observations from multiple poses, providing richer geometric constraints and generally improving short-term localization stability.

Local-map NDT is particularly useful for large-scale robotic operation. Rather than representing an entire environment at full resolution during every registration cycle, the system maintains NDT cells around the current robot location. As the platform moves, new regions are introduced and distant regions are removed, archived, or stored as submaps. This bounded representation supports predictable memory consumption and registration time while preserving nearby localization features.

Submap-based NDT extends local mapping by organizing the environment into independently manageable probabilistic map fragments. Each submap can contain NDT distributions created from multiple registered scans, while higher-level transformations connect the submaps into a global representation. Pose-graph optimization can subsequently adjust these transformations after loop closure or GNSS correction without requiring every raw LiDAR point to be globally reprocessed.

Several NDT variants modify how distributions are constructed or compared. Distribution-to-distribution approaches represent both source and target data statistically instead of treating the source solely as individual points. Other methods use overlapping cells, adaptive voxel sizes, hierarchical grids, or neighborhood distributions to reduce sensitivity to artificial voxel boundaries. These extensions seek to improve smoothness, accuracy, computational efficiency, or robustness across different environments.

Dynamic objects remain problematic because NDT assumes that local distributions describe persistent environmental structure. Vehicles, pedestrians, machinery, vegetation, and other changing elements can modify cell statistics or generate inconsistent likelihoods. Temporal filtering, occupancy persistence, semantic segmentation, motion detection, and robust weighting can prevent transient measurements from dominating the reference distributions used for localization.

Degenerate geometry can also weaken NDT pose estimation. A long tunnel, flat road, repetitive warehouse aisle, or large planar surface may produce distributions that strongly constrain some motion directions but provide little information in others. The Hessian, covariance, eigenvalues, or other observability indicators can be inspected to identify poorly constrained dimensions. Sensor fusion can then supply complementary information rather than accepting uncertain NDT updates as equally reliable.

NDT integrates naturally with multi-sensor state estimation. An IMU provides high-rate rotational and acceleration information, wheel encoders contribute short-term planar motion, and GNSS supplies globally referenced position where signals are available. NDT contributes geometric constraints derived from the surrounding environment. Extended Kalman filters, factor graphs, or nonlinear optimization can combine these measurements according to their uncertainty and temporal characteristics.

For outdoor AMRs, motion distortion must often be addressed before NDT registration. A rotating LiDAR acquires points at different times while the platform may be translating, turning, or traversing uneven terrain. IMU-assisted deskewing transforms measurements to a common reference time before building or querying NDT distributions. Without this correction, a distorted scan may disagree with the map even when the underlying pose prediction is otherwise accurate.

Computational performance depends on voxel lookup, distribution construction, objective evaluation, derivative calculation, and numerical optimization. Efficient grid indexing, voxel hashing, parallel processing, SIMD operations, multithreading, and GPU acceleration can reduce registration latency. Incremental map updates are also valuable because recomputing every Gaussian distribution whenever a new scan arrives would be unnecessarily expensive for continuously operating robotic systems.

Map updating requires careful statistical management. When new registered points enter an existing cell, its mean and covariance can be updated incrementally rather than recalculated from all historical measurements. However, unrestricted accumulation may cause outdated observations to dominate long-term statistics. Sliding windows, decay mechanisms, confidence measures, or map maintenance policies can help NDT representations adapt when industrial or outdoor environments change over time.

Registration quality should not be evaluated only by the final optimization score. A low numerical cost may still correspond to an incorrect pose in repetitive or weakly constrained environments. Practical systems therefore consider convergence state, pose increment, likelihood, number of contributing cells, geometric observability, predicted motion consistency, and disagreement with other sensors. These indicators allow the localization system to reject or down-weight suspicious NDT solutions.

Compared with ICP, NDT replaces explicit nearest-neighbor correspondence with a probabilistic spatial model, which can provide smooth optimization and efficient map interaction. ICP variants may offer highly precise surface alignment when reliable correspondences and normals are available, whereas NDT can be attractive for structured map-based localization and large point sets. The appropriate choice depends on environment geometry, initialization quality, map representation, hardware, and latency requirements.

In a complete LiDAR SLAM architecture, NDT normally serves as the front-end registration component rather than the entire SLAM solution. Its pose estimates and uncertainty information become constraints for a back-end estimator, while keyframes, loop closures, GNSS observations, and inertial measurements correct accumulated drift. The optimized trajectory can subsequently update submap poses and improve the reference representation used by future NDT registration.

NDT should therefore be viewed as both a registration algorithm and a probabilistic geometric representation connecting LiDAR measurements with state estimation. Its voxel distributions compress environmental structure, its optimization estimates robot motion, and its map architecture can scale from local localization to large-area SLAM. With suitable initialization, resolution management, dynamic filtering, uncertainty handling, and sensor fusion, NDT provides a practical foundation for robust LiDAR-based autonomy.

정규분포 변환(Normal Distributions Transform, NDT)은 LiDAR 위치 추정(Localization)과 SLAM에서 널리 사용되는 기하학적 정합(Geometric Registration) 방법이다. NDT는 두 스캔 사이의 개별 포인트를 직접 대응시키는 대신 기준 포인트 클라우드(Reference Point Cloud)를 국부 기하 구조(Local Geometry)의 연속적인 확률적 표현(Probabilistic Representation)으로 변환한다. 이후 정합 과정에서는 입력 스캔의 포인트가 이러한 공간 분포에서 높은 가능도(Likelihood)를 갖도록 하는 강체 변환(Rigid Transformation)을 추정한다.

NDT의 기본 개념은 3차원 공간을 셀(Cell) 또는 복셀(Voxel)의 집합으로 분할하는 것이다. 각각의 점유 복셀(Occupied Voxel)에 포함된 포인트는 독립적인 측정값으로만 저장되는 대신 통계적으로 요약된다. 평균 벡터(Mean Vector)는 관측값의 국부 중심(Local Center)을 나타내며 공분산 행렬(Covariance Matrix)은 공간적인 분포를 표현한다. 따라서 포인트가 존재하는 각각의 셀은 국부 환경 기하 구조를 나타내는 다변량 가우시안 분포(Multivariate Gaussian Distribution)로 근사할 수 있다.

이러한 통계적 표현(Statistical Representation)은 NDT를 고전적인 ICP와 같은 대응점 기반 방법(Correspondence-Driven Method)과 구별한다. ICP는 명시적인 점 대 점(Point-to-Point) 또는 점 대 표면(Point-to-Surface) 대응 관계를 반복적으로 탐색하지만 NDT는 변환된 스캔 포인트를 기준 지도(Reference Map)에 정의된 확률분포와 비교한다. 따라서 최적화 목적 함수(Optimization Objective)는 이산적인 최근접점 거리(Nearest-Neighbor Distance)의 집합이 아니라 분포 가능도(Distribution Likelihood)를 기반으로 하며 반복적인 정합 과정에서 명시적인 대응점 탐색에 대한 의존도를 줄일 수 있다.

각 NDT 셀 내부의 공분산 행렬은 유용한 기하학적 정보를 포함한다. 평면 벽(Planar Wall)에서 샘플링된 포인트는 표면 방향으로 큰 분산(Variance)을 갖고 표면에 수직인 방향으로 작은 분산을 갖는다. 선형 구조(Linear Structure)는 또 다른 특징적인 고유값 패턴(Eigenvalue Pattern)을 생성하며 불규칙한 체적 영역(Volumetric Region)은 보다 균형 잡힌 분포를 생성한다. 따라서 NDT는 모든 원시 포인트를 최적화에 개별적으로 참여시키지 않으면서도 통계 정보를 통해 국부 방향(Local Orientation)과 형상(Shape)을 암묵적으로 표현한다.

정합 과정(Registration Process)은 센서 또는 로봇 자세에 대한 초기 추정값(Initial Estimate)으로 시작한다. 현재 LiDAR 스캔의 각 포인트는 이 추정값에 따라 변환된 후 기준 표현(Reference Representation)의 해당 NDT 셀과 연결된다. 변환된 포인트의 확률 밀도(Probability Density)는 셀의 평균과 공분산을 이용하여 평가된다. 다수의 스캔 포인트에서 계산된 값은 스캔과 지도 사이의 전체적인 일치도를 나타내는 목적 함수(Objective Function)로 결합된다.

최적화(Optimization)는 변환된 스캔이 NDT 지도에서 갖는 가능도를 최대화하거나 이에 대응하는 음의 점수(Negative Score)를 최소화하도록 병진(Translation)과 회전(Rotation) 파라미터를 조정한다. 확률적 목적 함수로부터 그래디언트(Gradient)와 많은 구현에서 2차 정보(Second-Order Information)가 계산된다. 이후 뉴턴 계열(Newton-Type), 준뉴턴(Quasi-Newton) 또는 관련 수치 최적화(Numerical Optimization) 방법을 이용하여 변환이 수렴하거나 미리 정의된 종료 조건(Termination Condition)이 만족될 때까지 자세를 반복적으로 갱신한다.

ICP와 마찬가지로 NDT는 기본적으로 국소 정합(Local Registration) 기법이므로 신뢰성 높은 초기 자세 추정값의 영향을 크게 받는다. 이전 SLAM 자세, 휠 오도메트리(Wheel Odometry), IMU 적분(IMU Integration), GNSS 또는 등속도 예측(Constant-Velocity Prediction)을 이용하여 최적화기가 적절한 수렴 영역(Convergence Region)에서 시작하도록 할 수 있다. 초기화가 부정확하면 변환된 포인트가 관련 없는 분포에 들어가면서 잘못된 자세로 수렴하거나 안정적인 해를 얻지 못할 수 있다.

복셀 해상도(Voxel Resolution)는 NDT에서 가장 영향력이 큰 파라미터 중 하나이다. 큰 셀은 많은 포인트를 포함하여 비교적 큰 초기 정렬 오차를 허용할 수 있는 부드러운 분포를 생성하지만 세밀한 환경 정보를 억제할 수 있다. 작은 셀은 더 많은 기하학적 구조를 보존하여 정밀한 정합을 지원할 수 있지만 신뢰성 있는 공분산 추정에 필요한 측정값이 부족해질 수 있으며 보다 복잡한 최적화 지형(Optimization Landscape)을 형성할 수 있다.

다중 해상도 NDT(Multi-Resolution NDT)는 여러 복셀 크기에서 정합을 수행하여 이러한 상충 관계를 해결한다. 먼저 거친 분포(Coarse Distribution)를 사용하여 대규모 정렬을 추정하고 안정적인 자세 근사값을 얻는다. 이후 점진적으로 더 작은 셀을 사용하여 세부적인 기하 구조를 보존하면서 정합을 반복한다. 이러한 거친 단계에서 세밀한 단계로 진행하는 전략(Coarse-to-Fine Strategy)은 유효 수렴 영역을 확장하면서 최종 해 근처에서 고해상도 공간 정보가 제공하는 정확도를 유지할 수 있다.

NDT는 센서와 응용 분야에 따라 2차원 또는 3차원 형태로 사용할 수 있다. 2D NDT는 레이저 측정값을 평면 셀(Planar Cell)로 표현하며 대체로 평면 환경에서 동작하는 지상 로봇(Ground Robot)에 적합하다. 3D NDT는 3차원 가우시안 분포를 사용하여 체적 공간(Volumetric Space)을 모델링하며 현대적인 멀티빔 LiDAR(Multi-Beam LiDAR), 자율주행 차량(Autonomous Vehicle), 야외 자율이동로봇(Outdoor AMR), 광산 장비(Mining Machine), 복잡한 지형을 이동하는 로봇 등에 보다 적합하다.

스캔 대 스캔 NDT(Scan-to-Scan NDT)는 현재 LiDAR 측정값을 이전 스캔으로 생성된 표현과 정렬한다. 이러한 아키텍처는 비교적 단순하고 기준 데이터의 양을 제한할 수 있지만 각각의 추정값이 이전 정합 결과에 크게 의존하기 때문에 오차가 누적될 수 있다. 스캔 대 지도 NDT(Scan-to-Map NDT)는 현재 스캔을 여러 자세에서 획득한 관측값을 포함하는 로컬 지도(Local Map)에 정렬하며, 더 풍부한 기하 제약을 제공하여 일반적으로 단기 위치 추정 안정성을 향상시킨다.

로컬 지도 NDT(Local-Map NDT)는 대규모 로봇 운용에서 특히 유용하다. 매 정합 주기마다 전체 환경을 최대 해상도로 표현하는 대신 시스템은 현재 로봇 위치 주변의 NDT 셀을 유지한다. 플랫폼이 이동함에 따라 새로운 영역을 추가하고 멀어진 영역은 제거하거나 저장하거나 서브맵(Submap)으로 관리한다. 이러한 제한된 표현(Bounded Representation)은 주변의 위치 추정 특징(Localization Feature)을 유지하면서 예측 가능한 메모리 사용량과 정합 시간을 제공한다.

서브맵 기반 NDT(Submap-Based NDT)는 환경을 독립적으로 관리할 수 있는 확률적 지도 조각(Probabilistic Map Fragment)으로 구성하여 로컬 매핑(Local Mapping)을 확장한다. 각각의 서브맵은 여러 정합 스캔으로 생성된 NDT 분포를 포함할 수 있으며 상위 수준의 변환(Higher-Level Transformation)이 서브맵들을 하나의 전역 표현(Global Representation)으로 연결한다. 이후 루프 폐쇄(Loop Closure) 또는 GNSS 보정이 발생하면 포즈 그래프 최적화(Pose-Graph Optimization)를 통해 모든 원시 LiDAR 포인트를 다시 처리하지 않고도 이러한 변환을 조정할 수 있다.

여러 NDT 변형(NDT Variant)은 분포를 구성하거나 비교하는 방식을 변경한다. 분포 대 분포(Distribution-to-Distribution) 접근법은 소스 데이터를 개별 포인트로만 처리하는 대신 소스와 대상 데이터를 모두 통계적으로 표현한다. 다른 방법에서는 중첩 셀(Overlapping Cell), 적응형 복셀 크기(Adaptive Voxel Size), 계층형 그리드(Hierarchical Grid), 이웃 분포(Neighborhood Distribution) 등을 사용하여 인위적인 복셀 경계에 대한 민감도를 줄인다. 이러한 확장 방법은 다양한 환경에서 부드러움, 정확도, 계산 효율성 또는 강인성을 향상시키는 것을 목표로 한다.

동적 객체(Dynamic Object)는 NDT의 국부 분포가 지속적인 환경 구조(Persistent Environmental Structure)를 표현한다고 가정하기 때문에 여전히 문제가 된다. 차량, 보행자, 기계, 식생 등 변화하는 요소는 셀 통계를 변경하거나 일관되지 않은 가능도 값을 생성할 수 있다. 시간적 필터링(Temporal Filtering), 점유 지속성(Occupancy Persistence), 의미론적 분할(Semantic Segmentation), 운동 검출(Motion Detection), 강인 가중화(Robust Weighting)를 사용하면 일시적인 측정값이 위치 추정에 사용되는 기준 분포를 지배하는 것을 방지할 수 있다.

퇴화 기하 구조(Degenerate Geometry)는 NDT 자세 추정을 약화시킬 수도 있다. 긴 터널, 평평한 도로, 반복적인 창고 통로 또는 대형 평면 표면은 특정 운동 방향을 강하게 제약하면서 다른 방향에 대해서는 충분한 정보를 제공하지 못할 수 있다. 헤시안(Hessian), 공분산(Covariance), 고유값(Eigenvalue) 또는 기타 관측 가능성 지표(Observability Indicator)를 분석하여 제약이 약한 차원을 식별할 수 있다. 이후 불확실한 NDT 갱신값을 동일한 신뢰도로 사용하는 대신 센서 융합(Sensor Fusion)을 통해 보완 정보를 제공할 수 있다.

NDT는 다중 센서 상태 추정(Multi-Sensor State Estimation)과 자연스럽게 통합될 수 있다. IMU는 고주파 회전 및 가속도 정보를 제공하고 휠 인코더(Wheel Encoder)는 단기 평면 운동 정보를 제공하며 GNSS는 신호를 사용할 수 있는 환경에서 전역 기준 위치를 제공한다. NDT는 주변 환경에서 추출된 기하학적 제약을 제공한다. 확장 칼만 필터(Extended Kalman Filter), 팩터 그래프(Factor Graph), 비선형 최적화(Nonlinear Optimization)를 이용하면 각 센서의 불확실성과 시간적 특성에 따라 이러한 측정값을 결합할 수 있다.

야외 자율이동로봇(Outdoor AMR)의 경우 NDT 정합 이전에 모션 왜곡(Motion Distortion)을 처리해야 하는 경우가 많다. 회전형 LiDAR(Rotating LiDAR)는 플랫폼이 병진하거나 회전하거나 불규칙한 지형을 통과하는 동안 서로 다른 시간에 포인트를 획득한다. IMU 보조 디스큐잉(IMU-Assisted Deskewing)은 NDT 분포를 생성하거나 조회하기 전에 측정값을 공통 기준 시간(Common Reference Time)으로 변환한다. 이러한 보정이 없으면 기본적인 자세 예측이 정확하더라도 왜곡된 스캔이 지도와 일치하지 않을 수 있다.

계산 성능(Computational Performance)은 복셀 탐색(Voxel Lookup), 분포 생성(Distribution Construction), 목적 함수 평가(Objective Evaluation), 미분 계산(Derivative Calculation), 수치 최적화에 의해 결정된다. 효율적인 그리드 인덱싱(Grid Indexing), 복셀 해싱(Voxel Hashing), 병렬 처리(Parallel Processing), SIMD 연산, 멀티스레딩(Multithreading), GPU 가속(GPU Acceleration)을 이용하면 정합 지연 시간(Registration Latency)을 줄일 수 있다. 새로운 스캔이 입력될 때마다 모든 가우시안 분포를 다시 계산하는 것은 지속적으로 운용되는 로봇 시스템에서 비효율적이므로 증분 지도 갱신(Incremental Map Update) 역시 중요하다.

지도 갱신(Map Update)에는 세심한 통계 관리(Statistical Management)가 필요하다. 새롭게 정합된 포인트가 기존 셀에 입력될 때 모든 과거 측정값을 다시 계산하지 않고 평균과 공분산을 증분 방식(Incremental Method)으로 갱신할 수 있다. 그러나 제한 없는 누적은 오래된 관측값이 장기 통계를 지배하게 만들 수 있다. 슬라이딩 윈도(Sliding Window), 감쇠 메커니즘(Decay Mechanism), 신뢰도 척도(Confidence Measure), 지도 유지 정책(Map Maintenance Policy)을 적용하면 산업 또는 야외 환경이 시간에 따라 변화할 때 NDT 표현이 이에 적응하도록 할 수 있다.

정합 품질(Registration Quality)은 최종 최적화 점수만으로 평가해서는 안 된다. 반복적이거나 제약이 약한 환경에서는 낮은 수치 비용(Numerical Cost)이 잘못된 자세와 대응할 수도 있다. 따라서 실제 시스템에서는 수렴 상태(Convergence State), 자세 변화량(Pose Increment), 가능도, 기여 셀의 수, 기하학적 관측 가능성(Geometric Observability), 예측 운동과의 일관성, 다른 센서와의 불일치 등을 함께 고려한다. 이러한 지표를 이용하면 위치 추정 시스템이 의심스러운 NDT 결과를 거부하거나 낮은 가중치를 부여할 수 있다.

ICP와 비교하면 NDT는 명시적인 최근접 이웃 대응(Nearest-Neighbor Correspondence)을 확률적 공간 모델(Probabilistic Spatial Model)로 대체하므로 부드러운 최적화와 효율적인 지도 상호작용을 제공할 수 있다. ICP 변형은 신뢰성 높은 대응점과 표면 법선을 사용할 수 있을 때 매우 정밀한 표면 정합을 제공할 수 있으며, NDT는 구조화된 지도 기반 위치 추정(Map-Based Localization)과 대규모 포인트 집합에 적합할 수 있다. 적절한 방법은 환경 기하 구조, 초기화 품질, 지도 표현, 하드웨어, 지연 시간 요구사항에 따라 달라진다.

완전한 LiDAR SLAM 아키텍처에서 NDT는 일반적으로 전체 SLAM 자체라기보다 프론트엔드 정합 요소(Front-End Registration Component)의 역할을 수행한다. NDT의 자세 추정값과 불확실성 정보는 백엔드 추정기(Back-End Estimator)의 제약 조건이 되며 키프레임(Keyframe), 루프 폐쇄, GNSS 관측, 관성 측정(Inertial Measurement)을 통해 누적 드리프트를 보정한다. 이후 최적화된 궤적(Optimized Trajectory)을 이용하여 서브맵 자세를 갱신하고 이후 NDT 정합에 사용되는 기준 표현을 개선할 수 있다.

따라서 NDT는 LiDAR 측정과 상태 추정(State Estimation)을 연결하는 정합 알고리즘인 동시에 확률적 기하 표현(Probabilistic Geometric Representation)으로 이해할 수 있다. 복셀 분포(Voxel Distribution)는 환경 구조를 압축하고 최적화 과정은 로봇의 움직임을 추정하며 지도 아키텍처는 로컬 위치 추정에서 대규모 SLAM까지 확장될 수 있다. 적절한 초기화, 해상도 관리(Resolution Management), 동적 객체 필터링, 불확실성 처리, 센서 융합을 결합하면 NDT는 강인한 LiDAR 기반 자율 시스템(Robust LiDAR-Based Autonomy)을 구축하기 위한 실용적인 기반을 제공한다.

##  

## 06.04. HDL Graph SLAM Full 3D Loop Closure [w/Code]

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

HDL Graph SLAM is a graph-based three-dimensional SLAM architecture designed primarily for LiDAR-equipped mobile robots and vehicles. It combines sequential point-cloud registration with pose-graph optimization to construct globally consistent 3D maps over extended trajectories. Rather than treating each estimated pose independently, the system represents robot states as graph nodes and sensor-derived spatial relationships as edges that can be jointly optimized.

The front end begins with sequential 3D LiDAR scans acquired while the platform moves through the environment. Raw measurements typically require filtering, downsampling, and coordinate transformation before registration. Voxel-grid filtering reduces redundant points while retaining major geometric structures. Depending on the sensor and vehicle dynamics, motion compensation may also be required so that points acquired at different times are expressed in a consistent scan reference frame.

Scan matching estimates relative motion between successive LiDAR observations. HDL Graph SLAM commonly employs registration methods such as Normal Distributions Transform or Iterative Closest Point variants to determine the transformation between scans or between a scan and a local reference. The resulting translation and rotation estimate provides a relative-pose constraint that can be inserted into the graph rather than being used only as an irreversible incremental trajectory update.

A pose graph expresses the SLAM problem as a network of robot poses connected by spatial constraints. Each node typically represents the six-degree-of-freedom pose of a keyframe, including three-dimensional position and orientation. Edges encode measured transformations between nodes together with uncertainty or information matrices. Sequential scan-matching edges form the basic chain of the trajectory, while additional sensors and loop closures introduce complementary constraints.

Keyframes prevent the graph from growing unnecessarily with every LiDAR measurement. A new keyframe can be created when translation, rotation, time, or other motion criteria exceed predefined thresholds. The associated point cloud and estimated pose are stored for subsequent mapping and optimization. Selecting meaningful keyframes reduces computational load while preserving sufficient spatial information for registration, loop detection, and reconstruction of the surrounding environment.

A major advantage of graph-based SLAM is that historical poses remain adjustable. Pure incremental odometry integrates relative motion continuously, so small registration errors accumulate as trajectory drift. In a pose graph, sequential constraints can later be reconsidered together with newly discovered information. Nonlinear graph optimization searches for the set of node poses that best satisfies all available constraints, distributing corrections across the trajectory instead of applying them only to the latest state.

Full 3D optimization distinguishes this architecture from systems that assume motion occurs strictly on a two-dimensional plane. Robot states may contain translation along x, y, and z together with roll, pitch, and yaw. This capability is important for outdoor AMRs, autonomous vehicles, ramps, slopes, uneven industrial sites, underground environments, and multi-level facilities where terrain geometry or vehicle attitude cannot be represented accurately by planar motion alone.

Loop closure is essential for controlling long-term drift. When the robot returns to a previously visited region, the system attempts to determine whether the current observation corresponds to an earlier keyframe or map area. A valid loop closure creates an additional edge connecting graph nodes that may be separated by a long time interval. This new constraint provides information capable of correcting accumulated positional and rotational errors throughout the trajectory.

Loop candidates can be identified using spatial proximity, pose predictions, geometric descriptors, or place-recognition methods. Candidate generation alone is insufficient because perceptually similar environments may produce false matches. The associated point clouds therefore require geometric verification using registration. A loop constraint should be inserted only when alignment quality, overlap, transformation plausibility, and other validation criteria indicate that the observations represent the same physical location.

False loop closures can severely deform an otherwise valid map because graph optimization attempts to satisfy every accepted constraint. Robust kernels and consistency checks reduce the influence of questionable edges. Methods such as Huber-type losses can down-weight constraints with unexpectedly large residuals, while additional geometric validation can reject them before optimization. Reliable loop handling therefore requires both candidate detection and conservative verification.

Graph optimization is formulated as a nonlinear least-squares problem. For each edge, the transformation predicted from the connected node poses is compared with the transformation measured by registration or another sensor. The resulting residual is weighted by its information matrix, and the optimizer adjusts graph states to minimize the combined error. Libraries based on graph optimization frameworks can efficiently solve sparse systems containing large numbers of poses and constraints.

Information matrices determine how strongly individual edges influence the optimized solution. A precise LiDAR registration constraint should generally contribute more strongly than an uncertain measurement, while weak or degenerate alignment should contribute less. Estimating realistic uncertainty is therefore important. Registration residuals, local geometry, Hessian properties, covariance approximations, sensor characteristics, or empirical models can provide information about constraint reliability.

HDL Graph SLAM can incorporate multiple sensor modalities in addition to LiDAR. IMU measurements provide orientation and high-frequency motion information, GNSS contributes globally referenced position, and wheel odometry provides short-term motion constraints. These observations can be represented as graph edges, priors, or preprocessing inputs. Their complementary characteristics improve robustness when LiDAR geometry alone does not sufficiently constrain every degree of freedom.

GNSS constraints are particularly valuable for large outdoor maps because loop closures may occur infrequently over long routes. Global position measurements can limit accumulated drift and anchor the graph to a geographic coordinate system. However, GNSS accuracy varies significantly around buildings, vegetation, tunnels, and reflective structures. Measurements should therefore be weighted according to quality rather than being treated as exact global positions.

Floor or ground-plane constraints can also stabilize full 3D mapping when the operating environment provides meaningful planar structure. Estimated ground planes may constrain roll, pitch, or vertical motion and reduce gradual deformation of the trajectory. Such constraints must be applied carefully in environments containing steep slopes, ramps, rough terrain, or multiple floor orientations, where an overly rigid planar assumption could introduce systematic mapping errors.

After graph optimization changes keyframe poses, the global point-cloud map must reflect the corrected trajectory. Rather than permanently merging every measurement into a fixed cloud at acquisition time, keyframe point clouds can remain associated with their graph nodes. The map is reconstructed by transforming each stored cloud according to its optimized pose. This approach enables loop-closure corrections to propagate naturally into the final 3D environment representation.

Map generation introduces a tradeoff between geometric detail and computational cost. Combining all raw points can produce extremely dense maps that require large amounts of memory and storage. Voxel downsampling, local aggregation, keyframe selection, and resolution management reduce this burden. Different map products may also be generated for visualization, localization, obstacle avoidance, navigation, or offline inspection rather than requiring one representation to serve every purpose.

Dynamic environments create challenges because graph constraints should ideally be derived from static geometry. Moving vehicles, pedestrians, forklifts, machinery, vegetation, and temporary objects can corrupt registration or create misleading loop matches. Statistical filtering, temporal persistence, semantic segmentation, robust estimation, and local-map management can reduce their influence. Long-term deployment additionally requires distinguishing permanent structural changes from transient scene variation.

Degenerate geometry can cause apparently successful registration while leaving particular motion dimensions weakly constrained. Long corridors, tunnels, flat roads, and open spaces are common examples. If such a registration edge is assigned excessive confidence, graph optimization can propagate its error to other poses. Observability analysis and uncertainty-aware weighting allow the system to reduce reliance on weak LiDAR constraints and increase the contribution of IMU, GNSS, or wheel information.

The computational architecture typically separates high-frequency front-end processing from lower-frequency back-end optimization. LiDAR registration must provide poses rapidly enough for local navigation, while graph optimization and loop detection can operate asynchronously. This separation prevents expensive global corrections from blocking real-time perception and control. When an optimized trajectory becomes available, corrected poses and maps can be published without interrupting continuous scan processing.

Large-scale operation requires careful management of graph size and point-cloud storage. Thousands of keyframes and loop candidates can increase optimization, search, and memory costs. Spatial indexing, incremental optimization, submaps, selective loop searches, keyframe pruning, and hierarchical representations can improve scalability. The objective is to preserve globally useful constraints while avoiding computation on redundant measurements that contribute little new information.

HDL Graph SLAM therefore connects local geometric registration with global trajectory reasoning. The front end converts LiDAR observations into relative-motion constraints, while the graph back end maintains a revisable history of robot poses. Loop closure, GNSS, IMU, odometry, and structural constraints provide additional relationships that can correct accumulated drift. The optimized keyframe poses then become the foundation for reconstructing a globally consistent three-dimensional map.

For an outdoor or industrial AMR, successful deployment depends on more than obtaining an attractive point-cloud reconstruction. Localization continuity, loop-closure reliability, optimization latency, map consistency, sensor synchronization, recovery from registration failure, memory growth, and behavior under environmental change must all be considered. A well-designed full 3D graph-SLAM system provides a scalable framework in which local LiDAR accuracy and global spatial consistency can reinforce each other throughout long-term autonomous operation.

HDL 그래프 SLAM(HDL Graph SLAM)은 주로 LiDAR를 탑재한 이동 로봇(Mobile Robot)과 차량을 위해 설계된 그래프 기반 3차원 SLAM(Graph-Based 3D SLAM) 아키텍처이다. 연속적인 포인트 클라우드 정합(Point-Cloud Registration)과 포즈 그래프 최적화(Pose-Graph Optimization)를 결합하여 긴 이동 궤적에서도 전역적으로 일관된 3차원 지도(Globally Consistent 3D Map)를 생성한다. 각각의 추정 자세를 독립적으로 처리하지 않고 로봇 상태를 그래프 노드(Graph Node), 센서로부터 얻어진 공간적 관계를 에지(Edge)로 표현하여 전체적으로 최적화한다.

프론트엔드(Front End)는 플랫폼이 환경을 이동하는 동안 연속적으로 획득되는 3차원 LiDAR 스캔에서 시작한다. 원시 측정값(Raw Measurement)은 일반적으로 정합 전에 필터링(Filtering), 다운샘플링(Downsampling), 좌표 변환(Coordinate Transformation)이 필요하다. 복셀 그리드 필터링(Voxel-Grid Filtering)은 주요 기하 구조를 유지하면서 중복 포인트를 감소시킨다. 센서 특성과 차량 동역학에 따라 서로 다른 시간에 획득된 포인트를 일관된 스캔 기준 좌표계로 표현하기 위한 모션 보상(Motion Compensation)도 필요할 수 있다.

스캔 정합(Scan Matching)은 연속적인 LiDAR 관측 사이의 상대 움직임(Relative Motion)을 추정한다. HDL Graph SLAM에서는 일반적으로 정규분포 변환(Normal Distributions Transform, NDT)이나 반복 최근접점(Iterative Closest Point, ICP) 계열의 정합 방법을 이용하여 스캔 사이 또는 스캔과 로컬 기준(Local Reference) 사이의 변환을 계산한다. 이렇게 얻어진 병진과 회전 추정값은 단순히 되돌릴 수 없는 증분 궤적 갱신에만 사용되지 않고 그래프에 삽입할 수 있는 상대 자세 제약(Relative-Pose Constraint)을 제공한다.

포즈 그래프(Pose Graph)는 SLAM 문제를 공간 제약으로 연결된 로봇 자세의 네트워크로 표현한다. 각각의 노드는 일반적으로 3차원 위치와 자세를 포함하는 키프레임(Keyframe)의 6자유도 자세(Six-Degree-of-Freedom Pose)를 나타낸다. 에지는 노드 사이에서 측정된 변환과 이에 대응하는 불확실성(Uncertainty) 또는 정보 행렬(Information Matrix)을 표현한다. 연속적인 스캔 정합 에지는 궤적의 기본 연결 구조를 형성하며 추가 센서와 루프 폐쇄(Loop Closure)는 보완적인 제약을 제공한다.

키프레임은 모든 LiDAR 측정값에 대해 그래프가 불필요하게 증가하는 것을 방지한다. 병진, 회전, 시간 또는 다른 운동 조건이 사전에 정의된 임계값을 초과하면 새로운 키프레임을 생성할 수 있다. 관련 포인트 클라우드와 추정 자세는 이후의 매핑(Mapping)과 최적화를 위해 저장된다. 의미 있는 키프레임을 선택하면 주변 환경의 정합, 루프 검출(Loop Detection), 재구성에 필요한 충분한 공간 정보를 유지하면서 계산 부하를 감소시킬 수 있다.

그래프 기반 SLAM(Graph-Based SLAM)의 주요 장점은 과거 자세(Historical Pose)를 이후에도 수정할 수 있다는 점이다. 순수 증분 오도메트리(Incremental Odometry)는 상대 운동을 지속적으로 적분하기 때문에 작은 정합 오차가 궤적 드리프트(Trajectory Drift)로 누적된다. 포즈 그래프에서는 새롭게 발견된 정보와 함께 이전의 연속 제약을 다시 고려할 수 있다. 비선형 그래프 최적화(Nonlinear Graph Optimization)는 모든 가용 제약을 가장 잘 만족하는 노드 자세의 집합을 탐색하여 최신 상태에만 보정을 적용하지 않고 전체 궤적에 걸쳐 오차를 분산시킨다.

완전한 3차원 최적화(Full 3D Optimization)는 로봇의 움직임이 엄격하게 2차원 평면에서 발생한다고 가정하는 시스템과 이 아키텍처를 구분하는 중요한 특징이다. 로봇 상태는 x, y, z 방향의 병진과 롤(Roll), 피치(Pitch), 요(Yaw)를 포함할 수 있다. 이러한 기능은 야외 자율이동로봇(Outdoor AMR), 자율주행 차량, 경사로, 비탈길, 불규칙한 산업 현장, 지하 환경, 다층 시설처럼 지형의 기하 구조나 차량 자세를 평면 운동만으로 정확하게 표현하기 어려운 환경에서 중요하다.

루프 폐쇄(Loop Closure)는 장기적인 드리프트를 제어하기 위한 핵심 기능이다. 로봇이 이전에 방문했던 영역으로 돌아오면 시스템은 현재 관측값이 과거의 키프레임이나 지도 영역과 대응하는지를 판단한다. 유효한 루프 폐쇄가 검출되면 시간적으로 멀리 떨어져 있는 그래프 노드 사이에 추가적인 에지가 생성된다. 이러한 새로운 제약은 전체 궤적에서 누적된 위치 및 회전 오차를 보정할 수 있는 정보를 제공한다.

루프 후보(Loop Candidate)는 공간적 근접성(Spatial Proximity), 자세 예측(Pose Prediction), 기하학적 디스크립터(Geometric Descriptor), 장소 인식(Place Recognition) 등을 이용하여 탐색할 수 있다. 그러나 지각적으로 유사한 환경이 잘못된 대응(False Match)을 생성할 수 있으므로 후보 생성만으로는 충분하지 않다. 따라서 관련 포인트 클라우드를 정합하여 기하학적으로 검증해야 한다. 정렬 품질, 중첩도(Overlap), 변환의 타당성(Transformation Plausibility) 등의 검증 조건이 두 관측값이 동일한 물리적 위치를 나타낸다고 판단할 때만 루프 제약을 삽입해야 한다.

잘못된 루프 폐쇄(False Loop Closure)는 그래프 최적화가 수용된 모든 제약을 만족시키려고 하기 때문에 정상적인 지도를 심각하게 변형시킬 수 있다. 강인 커널(Robust Kernel)과 일관성 검사(Consistency Check)는 의심스러운 에지의 영향을 줄인다. 후버 계열 손실(Huber-Type Loss)과 같은 방법은 예상보다 큰 잔차를 갖는 제약의 가중치를 감소시킬 수 있으며 추가적인 기하 검증을 통해 최적화 전에 잘못된 제약을 제거할 수도 있다. 따라서 신뢰성 높은 루프 처리는 후보 검출과 보수적인 검증을 모두 필요로 한다.

그래프 최적화(Graph Optimization)는 비선형 최소제곱 문제(Nonlinear Least-Squares Problem)로 구성된다. 각 에지에 대해 연결된 노드 자세로부터 예측되는 변환과 정합 또는 다른 센서에서 측정된 변환을 비교한다. 그 결과로 얻어진 잔차(Residual)는 정보 행렬에 의해 가중되며 최적화기는 모든 제약의 결합 오차를 최소화하도록 그래프 상태를 조정한다. 그래프 최적화 프레임워크(Graph Optimization Framework)를 기반으로 하는 라이브러리는 많은 자세와 제약으로 구성된 희소 시스템(Sparse System)을 효율적으로 해결할 수 있다.

정보 행렬(Information Matrix)은 각각의 에지가 최적화된 해에 얼마나 강하게 영향을 미치는지를 결정한다. 정밀한 LiDAR 정합 제약은 일반적으로 불확실한 측정보다 더 큰 영향을 주어야 하며, 약하거나 퇴화된 정합(Degenerate Registration)은 상대적으로 낮은 영향을 가져야 한다. 따라서 현실적인 불확실성 추정(Uncertainty Estimation)이 중요하다. 정합 잔차, 국부 기하 구조, 헤시안(Hessian) 특성, 공분산 근사(Covariance Approximation), 센서 특성 또는 경험적 모델을 통해 제약의 신뢰도 정보를 얻을 수 있다.

HDL Graph SLAM은 LiDAR 외에도 여러 센서 모달리티(Sensor Modality)를 통합할 수 있다. IMU 측정값은 자세와 고주파 운동 정보를 제공하고 GNSS는 전역 기준 위치(Global Position)를 제공하며 휠 오도메트리는 단기 운동 제약을 제공한다. 이러한 관측값은 그래프 에지, 사전 제약(Prior), 또는 전처리 입력으로 표현할 수 있다. 서로 보완적인 센서 특성은 LiDAR 기하 구조만으로 모든 자유도를 충분히 제약할 수 없는 상황에서 강인성을 향상시킨다.

GNSS 제약(GNSS Constraint)은 긴 경로에서 루프 폐쇄가 자주 발생하지 않을 수 있기 때문에 대규모 야외 지도에서 특히 중요하다. 전역 위치 측정값은 누적 드리프트를 제한하고 그래프를 지리적 좌표계(Geographic Coordinate System)에 고정할 수 있다. 그러나 GNSS 정확도는 건물, 식생, 터널, 반사 구조 주변에서 크게 변할 수 있다. 따라서 GNSS 측정값을 정확한 절대 위치로 취급하기보다 품질에 따라 적절한 가중치를 부여해야 한다.

바닥 또는 지면 평면 제약(Floor or Ground-Plane Constraint)은 운용 환경에 의미 있는 평면 구조가 존재할 경우 완전한 3차원 매핑을 안정화할 수 있다. 추정된 지면 평면은 롤, 피치 또는 수직 방향 움직임을 제약하여 궤적의 점진적인 변형을 줄일 수 있다. 그러나 급경사, 경사로, 거친 지형 또는 서로 다른 방향의 바닥이 존재하는 환경에서는 지나치게 엄격한 평면 가정(Planar Assumption)이 체계적인 매핑 오차를 발생시킬 수 있으므로 신중하게 적용해야 한다.

그래프 최적화가 키프레임 자세를 변경하면 전역 포인트 클라우드 지도(Global Point-Cloud Map)도 보정된 궤적을 반영해야 한다. 모든 측정값을 획득 시점에 하나의 고정된 클라우드로 영구적으로 병합하는 대신 각각의 키프레임 포인트 클라우드를 해당 그래프 노드와 연결된 상태로 유지할 수 있다. 이후 저장된 각 클라우드를 최적화된 자세에 따라 변환하여 지도를 재구성한다. 이를 통해 루프 폐쇄에 의한 보정이 최종 3차원 환경 표현에 자연스럽게 반영된다.

지도 생성(Map Generation)에서는 기하학적 세부 정보와 계산 비용 사이의 균형이 필요하다. 모든 원시 포인트를 결합하면 매우 높은 밀도의 지도가 생성되어 많은 메모리와 저장 공간을 필요로 한다. 복셀 다운샘플링(Voxel Downsampling), 로컬 집계(Local Aggregation), 키프레임 선택, 해상도 관리(Resolution Management)를 이용하면 이러한 부담을 줄일 수 있다. 또한 하나의 표현으로 모든 목적을 충족하기보다 시각화, 위치 추정, 장애물 회피, 내비게이션, 오프라인 검사 등에 맞는 서로 다른 지도 표현을 생성할 수 있다.

동적 환경(Dynamic Environment)은 그래프 제약이 이상적으로 정적 기하 구조(Static Geometry)에서 생성되어야 한다는 점에서 문제를 발생시킨다. 이동 차량, 보행자, 지게차, 기계, 식생, 임시 객체 등은 정합을 방해하거나 잘못된 루프 대응을 생성할 수 있다. 통계적 필터링(Statistical Filtering), 시간적 지속성(Temporal Persistence), 의미론적 분할(Semantic Segmentation), 강인 추정(Robust Estimation), 로컬 지도 관리를 통해 이러한 영향을 줄일 수 있다. 장기 운용에서는 영구적인 구조 변화와 일시적인 장면 변화를 구별하는 것도 필요하다.

퇴화 기하 구조(Degenerate Geometry)는 겉보기에는 성공적인 정합을 생성하면서도 특정 운동 차원에 대해서는 제약이 약할 수 있다. 긴 복도, 터널, 평평한 도로, 개방 공간 등이 대표적인 사례이다. 이러한 정합 에지에 지나치게 높은 신뢰도를 부여하면 그래프 최적화가 해당 오차를 다른 자세로 전파할 수 있다. 관측 가능성 분석(Observability Analysis)과 불확실성 기반 가중화(Uncertainty-Aware Weighting)를 이용하면 약한 LiDAR 제약에 대한 의존도를 낮추고 IMU, GNSS 또는 휠 정보의 기여도를 높일 수 있다.

계산 아키텍처(Computational Architecture)는 일반적으로 고주파 프론트엔드 처리와 상대적으로 저주파인 백엔드 최적화를 분리한다. LiDAR 정합은 로컬 내비게이션(Local Navigation)에 사용할 수 있을 정도로 빠르게 자세를 제공해야 하지만 그래프 최적화와 루프 검출은 비동기 방식(Asynchronous Processing)으로 수행할 수 있다. 이러한 분리는 계산량이 큰 전역 보정(Global Correction)이 실시간 인지와 제어를 차단하는 것을 방지한다. 최적화된 궤적이 준비되면 지속적인 스캔 처리를 중단하지 않고 보정된 자세와 지도를 제공할 수 있다.

대규모 운용(Large-Scale Operation)에서는 그래프 크기와 포인트 클라우드 저장 공간을 세심하게 관리해야 한다. 수천 개의 키프레임과 루프 후보가 생성되면 최적화, 탐색, 메모리 비용이 증가할 수 있다. 공간 인덱싱(Spatial Indexing), 증분 최적화(Incremental Optimization), 서브맵(Submap), 선택적 루프 탐색(Selective Loop Search), 키프레임 가지치기(Keyframe Pruning), 계층적 표현(Hierarchical Representation)을 이용하면 확장성을 향상시킬 수 있다. 목표는 전역적으로 유용한 제약을 보존하면서 새로운 정보를 거의 제공하지 않는 중복 측정에 대한 계산을 줄이는 것이다.

따라서 HDL Graph SLAM은 로컬 기하 정합(Local Geometric Registration)과 전역 궤적 추론(Global Trajectory Reasoning)을 연결한다. 프론트엔드는 LiDAR 관측을 상대 운동 제약으로 변환하고 그래프 백엔드(Graph Back End)는 수정 가능한 로봇 자세의 이력을 유지한다. 루프 폐쇄, GNSS, IMU, 오도메트리, 구조적 제약(Structural Constraint)은 누적 드리프트를 보정할 수 있는 추가 관계를 제공한다. 이후 최적화된 키프레임 자세는 전역적으로 일관된 3차원 지도를 재구성하는 기반이 된다.

야외 또는 산업용 자율이동로봇(Outdoor or Industrial AMR)에 성공적으로 적용하려면 시각적으로 우수한 포인트 클라우드 지도를 생성하는 것 이상의 요소를 고려해야 한다. 위치 추정 연속성(Localization Continuity), 루프 폐쇄 신뢰성, 최적화 지연 시간(Optimization Latency), 지도 일관성, 센서 동기화(Sensor Synchronization), 정합 실패 이후의 복구, 메모리 증가, 환경 변화에 대한 동작을 함께 평가해야 한다. 잘 설계된 완전한 3차원 그래프 SLAM(Full 3D Graph SLAM)은 장기 자율 운용 과정에서 로컬 LiDAR 정확도와 전역 공간 일관성이 서로 강화될 수 있는 확장 가능한 프레임워크를 제공한다.

##  

## 06.05. Faster LIO Incremental LiDAR Inertial Odometry [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

Faster-LIO is a tightly coupled LiDAR-inertial odometry framework designed for high-rate, accurate state estimation from 3D LiDAR and IMU measurements. Its central objective is to estimate robot motion incrementally while continuously maintaining a local geometric map. By combining high-frequency inertial prediction with LiDAR geometric correction, the system can remain responsive during rapid translation, rotation, and complex six-degree-of-freedom motion.

LiDAR and IMU provide complementary information. A LiDAR directly measures surrounding geometry but normally operates at a lower frequency and collects points over a finite scan interval. An IMU produces high-rate angular velocity and linear acceleration but accumulates drift when integrated over time. Faster-LIO combines these sensors so that inertial measurements predict short-term motion while LiDAR observations repeatedly constrain accumulated state error using environmental geometry.

Accurate temporal synchronization is fundamental to LiDAR-inertial odometry. Individual LiDAR points may be measured at different times during a scan, while IMU samples arrive much more frequently. The estimator must associate these measurements with a consistent time base. Even small timestamp offsets can create systematic registration errors during rapid motion, so hardware synchronization or carefully calibrated software timing is highly desirable for reliable operation.

IMU preprocessing provides the motion information required between LiDAR updates. Angular velocity and acceleration measurements are integrated to propagate orientation, velocity, and position while accounting for gravity and sensor biases. This propagation generates a predicted state at the time associated with each LiDAR measurement. The prediction is not treated as permanently correct because accelerometer and gyroscope biases gradually produce drift that must be corrected by geometric observations.

A rotating or scanning LiDAR does not capture the complete point cloud at one instant. If the robot moves during acquisition, different points are expressed from slightly different sensor poses, creating motion distortion. Faster-LIO uses IMU-derived motion estimates to deskew the point cloud, transforming measurements toward a common reference time. This produces a geometrically coherent scan that can be compared more reliably with the maintained local map.

Extrinsic calibration defines the rigid transformation between the LiDAR and IMU coordinate frames. Errors in relative translation or rotation directly affect motion compensation and geometric residuals, especially during strong rotational motion. Accurate calibration is therefore essential for tightly coupled estimation. Some LiDAR-inertial systems can refine extrinsic parameters online, while others rely on carefully measured and validated offline calibration before autonomous operation.

After motion compensation, LiDAR points are transformed into the predicted world or map frame and compared with nearby map geometry. Rather than aligning two complete point clouds through a conventional standalone ICP procedure, LiDAR-inertial odometry can formulate geometric residuals directly between selected scan points and local surface structures. These residuals measure disagreement between the inertially predicted pose and the geometry already represented in the local map.

Local planar structures are particularly useful for geometric correction. Neighboring map points around a transformed LiDAR measurement can be retrieved and used to estimate a plane. The point-to-plane distance becomes a residual that constrains the robot state. Measurements associated with poorly defined neighborhoods, excessive residuals, dynamic objects, or unreliable geometry can be rejected so that unstable correspondences do not dominate the estimator.

The estimator jointly reasons about position, orientation, velocity, gravity-related quantities, and IMU biases rather than estimating only a rigid scan transformation. In tightly coupled formulations, LiDAR residuals directly update the navigation state predicted by the IMU. This differs conceptually from loosely coupled architectures in which a separate LiDAR odometry module first generates poses that are subsequently fused with inertial estimates.

Iterated Kalman filtering is commonly associated with modern tightly coupled LiDAR-inertial odometry. An initial state predicted from IMU propagation is repeatedly linearized against LiDAR geometric measurements. Each iteration computes corrections that reduce residuals between transformed scan points and map surfaces. Re-linearization around the updated state improves estimation when measurement models are nonlinear and the initial prediction is not sufficiently close to the final solution.

A major contribution of Faster-LIO is its emphasis on efficient incremental spatial data structures for local mapping and nearest-neighbor queries. Conventional point-cloud maps can become computationally expensive when every incoming scan requires repeated searches, insertion, and deletion. Efficient voxel-based organization allows the estimator to access nearby geometric information rapidly while controlling the number of stored points and the cost of continuously updating the map.

Voxel representations divide space into discrete three-dimensional cells and associate local map information with occupied regions. Hash-based or similarly efficient indexing can provide approximately direct access to relevant cells without searching an entire point cloud. This organization is well suited to online LiDAR processing because new observations can be inserted incrementally, nearby points can be queried efficiently, and distant or unnecessary map regions can be managed independently.

Incremental mapping means that the map evolves continuously as the robot moves. After the current state has been corrected, accepted LiDAR points are transformed into the map frame and inserted into the local representation. Redundant measurements may be removed or downsampled to prevent uncontrolled growth. The updated map then becomes the geometric reference for subsequent scans, creating a continuous prediction-correction-map-update cycle.

A local map is generally preferable to an indefinitely expanding global point cloud for real-time odometry. Only geometry surrounding the robot is required for immediate scan-to-map constraints. As the platform moves, new spatial regions enter the active map while distant regions can be discarded, archived, or handled separately. Bounding the active map helps maintain predictable memory use and nearest-neighbor search performance during long trajectories.

Faster-LIO is fundamentally an odometry and local mapping system, so long-term global consistency must be distinguished from short-term estimation accuracy. Incremental LiDAR-inertial estimation can substantially reduce drift but cannot guarantee a globally drift-free trajectory over arbitrarily long operation. Loop closure, GNSS constraints, pose-graph optimization, or other global corrections can be added at a higher level when globally consistent maps are required.

The tight coupling between LiDAR and IMU provides important advantages during aggressive motion. IMU propagation supplies high-frequency pose prediction when consecutive LiDAR scans have significant displacement, while LiDAR geometry prevents inertial integration from drifting without bound. This relationship also improves deskewing because the same motion estimate used for state propagation can describe sensor motion throughout the LiDAR acquisition interval.

Nevertheless, LiDAR-inertial estimation remains sensitive to observability. Long corridors, flat roads, tunnels, large walls, or open environments may provide weak geometric constraints along particular directions. In such situations, some state variables become difficult to estimate from LiDAR geometry alone. IMU information helps preserve short-term continuity, but external measurements such as GNSS, wheel odometry, or additional perception may be required for stronger long-term constraints.

IMU initialization is another important stage because gravity direction, initial orientation, velocity, and sensor biases influence subsequent propagation. If initialization occurs while the platform is stationary, gravity and bias-related quantities can often be estimated more reliably. Systems intended to initialize during motion require more sophisticated observability handling. Poor initialization can produce distorted deskewing, inconsistent map geometry, and slow or unstable convergence.

Dynamic objects also violate the assumption that geometric residuals originate from a static environment. Vehicles, pedestrians, forklifts, vegetation, and moving machinery can introduce map points that later become inconsistent. Residual thresholds, neighborhood checks, temporal filtering, semantic processing, or map-aging strategies can reduce their influence. Industrial deployment should therefore consider not only estimator accuracy but also the persistence policy of the local geometric map.

Computational efficiency is critical because IMU propagation, deskewing, neighborhood search, residual construction, iterative state updates, and map insertion must all occur within the LiDAR frame period. Faster-LIO targets this problem by reducing expensive spatial operations and maintaining an incrementally updated map. Efficient CPU implementation can therefore support high-rate estimation without requiring every stage to depend on computationally heavy global registration.

Parameter selection affects both accuracy and real-time performance. Voxel size determines map density and geometric detail, neighborhood settings influence plane estimation, residual thresholds control correspondence rejection, and local-map dimensions affect memory and search cost. Excessively dense maps may increase computation without improving observability, whereas overly coarse representations can remove structures required for accurate geometric correction.

Sensor characteristics should also influence configuration. A high-channel-count spinning LiDAR produces dense observations but may impose substantial processing load, while solid-state or non-repetitive scanning sensors generate different spatial sampling patterns. IMU noise density, bias stability, measurement frequency, LiDAR range accuracy, and field of view all affect estimator behavior. Parameters should therefore be validated using the actual sensor configuration and expected vehicle dynamics.

The state estimate produced by Faster-LIO can support downstream autonomous functions such as local navigation, obstacle mapping, motion planning, and sensor-frame transformation. High-rate pose output is particularly valuable when perception modules must transform asynchronous observations into a common world frame. However, safety-critical control should also monitor estimator health, timestamp validity, innovation magnitude, and sensor availability rather than assuming every published pose is equally reliable.

For outdoor AMRs, rough terrain introduces simultaneous translation, roll, pitch, vibration, and rapid changes in vertical acceleration. These conditions make planar odometry assumptions inadequate and increase the value of full 3D LiDAR-inertial estimation. Mechanical vibration isolation, rigid sensor mounting, synchronization, calibration, and appropriate IMU quality become important because software estimation cannot completely compensate for unstable hardware integration or corrupted measurements.

A practical architecture can use Faster-LIO as the high-rate local state-estimation layer and connect it to a slower global localization layer. GNSS or RTK can provide geographic anchoring outdoors, while loop closure and pose-graph optimization can correct accumulated drift in repeated areas. The local estimator continues producing smooth real-time motion estimates while the global layer manages long-term consistency, map alignment, and mission-level coordinates.

Faster-LIO therefore represents more than a faster point-cloud registration algorithm. Its essential concept is the efficient integration of inertial propagation, LiDAR motion compensation, geometric measurement updates, and incremental map management within one tightly coupled estimation cycle. This architecture allows local geometry and high-frequency inertial motion to continuously correct each other while keeping computational cost suitable for real-time robotic operation.

When deployed correctly, the framework provides a strong foundation for high-rate six-degree-of-freedom localization in autonomous vehicles and mobile robots. Its practical performance depends on synchronization, calibration, initialization, sensor quality, environmental observability, map parameters, and computational resources. Combined with an appropriate global correction layer, incremental LiDAR-inertial odometry can form the real-time localization backbone of scalable outdoor and industrial autonomous systems.

Faster-LIO는 3차원 LiDAR와 IMU 측정값으로부터 고속·고정밀 상태 추정(State Estimation)을 수행하도록 설계된 긴밀 결합형 LiDAR-관성 오도메트리(Tightly Coupled LiDAR-Inertial Odometry) 프레임워크이다. 핵심 목적은 로봇의 움직임을 증분 방식(Incremental Method)으로 추정하면서 로컬 기하 지도(Local Geometric Map)를 지속적으로 유지하는 것이다. 고주파 관성 예측(Inertial Prediction)과 LiDAR 기하 보정(Geometric Correction)을 결합하여 빠른 병진, 회전 및 복잡한 6자유도 운동(Six-Degree-of-Freedom Motion)에서도 신속한 상태 추정을 유지할 수 있다.

LiDAR와 IMU는 서로 보완적인 정보를 제공한다. LiDAR는 주변 기하 구조를 직접 측정하지만 일반적으로 상대적으로 낮은 주파수로 동작하며 일정한 스캔 시간 동안 포인트를 수집한다. IMU는 고주파 각속도(Angular Velocity)와 선형 가속도(Linear Acceleration)를 제공하지만 시간에 따라 적분하면 드리프트가 누적된다. Faster-LIO는 관성 측정값으로 단기 움직임을 예측하고 LiDAR 관측으로 환경 기하 구조를 이용하여 누적된 상태 오차를 반복적으로 보정한다.

정확한 시간 동기화(Temporal Synchronization)는 LiDAR-관성 오도메트리에서 핵심적인 요소이다. 각각의 LiDAR 포인트는 하나의 스캔이 진행되는 동안 서로 다른 시간에 측정될 수 있으며 IMU 샘플은 훨씬 높은 주파수로 입력된다. 추정기(Estimator)는 이러한 측정값을 일관된 시간 기준(Time Base)에 연결해야 한다. 빠른 움직임에서는 작은 타임스탬프 오프셋(Timestamp Offset)도 체계적인 정합 오차를 발생시킬 수 있으므로 신뢰성 높은 운용을 위해 하드웨어 동기화(Hardware Synchronization) 또는 정밀하게 보정된 소프트웨어 시간 동기화가 바람직하다.

IMU 전처리(IMU Preprocessing)는 LiDAR 갱신 사이의 움직임 정보를 제공한다. 각속도와 가속도 측정값을 적분하여 중력(Gravity)과 센서 바이어스(Sensor Bias)를 고려하면서 자세, 속도, 위치를 전파한다. 이러한 전파 과정은 각 LiDAR 측정값에 대응하는 시간에서 예측 상태(Predicted State)를 생성한다. 가속도계와 자이로스코프의 바이어스가 점진적인 드리프트를 발생시키므로 예측값을 영구적으로 정확한 상태로 간주하지 않고 기하학적 관측을 이용하여 지속적으로 보정한다.

회전형 또는 스캐닝 LiDAR(Scanning LiDAR)는 전체 포인트 클라우드를 한순간에 획득하지 않는다. 데이터 획득 중 로봇이 움직이면 각 포인트는 조금씩 다른 센서 자세에서 측정되어 모션 왜곡(Motion Distortion)이 발생한다. Faster-LIO는 IMU 기반 운동 추정값을 이용하여 포인트 클라우드를 디스큐잉(Deskewing)하고 측정값을 공통 기준 시간(Common Reference Time)으로 변환한다. 이를 통해 로컬 지도와 더욱 신뢰성 있게 비교할 수 있는 기하학적으로 일관된 스캔을 생성한다.

외부 파라미터 보정(Extrinsic Calibration)은 LiDAR와 IMU 좌표계 사이의 강체 변환(Rigid Transformation)을 정의한다. 상대 병진 또는 회전의 오차는 특히 큰 회전 운동이 발생할 때 모션 보상과 기하 잔차(Geometric Residual)에 직접적인 영향을 준다. 따라서 긴밀 결합형 추정에서는 정확한 보정이 필수적이다. 일부 LiDAR-관성 시스템은 외부 파라미터를 온라인으로 보정할 수 있지만 다른 시스템에서는 자율 운용 전에 정밀하게 측정하고 검증한 오프라인 보정(Offline Calibration)을 사용한다.

모션 보상 이후 LiDAR 포인트는 예측된 월드 또는 지도 좌표계(World or Map Frame)로 변환되어 주변 지도 기하 구조와 비교된다. 두 개의 전체 포인트 클라우드를 독립적인 기존 ICP 절차로 정렬하는 대신 LiDAR-관성 오도메트리는 선택된 스캔 포인트와 로컬 표면 구조 사이의 기하 잔차를 직접 구성할 수 있다. 이러한 잔차는 관성 기반으로 예측된 자세와 로컬 지도에 이미 표현된 기하 구조 사이의 불일치를 측정한다.

국부 평면 구조(Local Planar Structure)는 기하 보정에 특히 유용하다. 변환된 LiDAR 측정값 주변의 지도 포인트를 검색하여 평면을 추정할 수 있으며 포인트 대 평면 거리(Point-to-Plane Distance)가 로봇 상태를 제약하는 잔차가 된다. 명확하게 정의되지 않은 주변 구조, 과도하게 큰 잔차, 동적 객체(Dynamic Object), 신뢰성이 낮은 기하 구조와 연결된 측정값을 제거하여 불안정한 대응 관계가 추정기를 지배하는 것을 방지할 수 있다.

추정기는 단순한 강체 스캔 변환만을 계산하는 대신 위치, 자세, 속도, 중력 관련 상태량, IMU 바이어스 등을 함께 추정한다. 긴밀 결합형 구조(Tightly Coupled Architecture)에서는 LiDAR 잔차가 IMU에서 예측된 항법 상태(Navigation State)를 직접 갱신한다. 이는 별도의 LiDAR 오도메트리 모듈이 먼저 자세를 생성하고 이후 관성 추정값과 융합하는 느슨한 결합형 구조(Loosely Coupled Architecture)와 개념적으로 구분된다.

반복 칼만 필터링(Iterated Kalman Filtering)은 현대적인 긴밀 결합형 LiDAR-관성 오도메트리에서 일반적으로 사용되는 접근법이다. IMU 전파로 얻은 초기 상태를 LiDAR 기하 측정값에 대해 반복적으로 선형화(Linearization)한다. 각각의 반복 과정에서 변환된 스캔 포인트와 지도 표면 사이의 잔차를 감소시키는 보정값을 계산한다. 갱신된 상태 주변에서 다시 선형화하면 측정 모델이 비선형이고 초기 예측값이 최종 해에 충분히 가깝지 않은 경우에도 추정 성능을 향상시킬 수 있다.

Faster-LIO의 주요 특징 중 하나는 로컬 매핑(Local Mapping)과 최근접 이웃 탐색(Nearest-Neighbor Query)을 위한 효율적인 증분 공간 데이터 구조(Incremental Spatial Data Structure)를 강조한다는 점이다. 기존 포인트 클라우드 지도는 새로운 스캔마다 반복적인 탐색, 삽입, 삭제가 필요하여 계산 비용이 증가할 수 있다. 효율적인 복셀 기반 구성(Voxel-Based Organization)은 저장되는 포인트 수와 지속적인 지도 갱신 비용을 제어하면서 주변 기하 정보에 빠르게 접근할 수 있도록 한다.

복셀 표현(Voxel Representation)은 공간을 이산적인 3차원 셀로 분할하고 점유된 영역에 로컬 지도 정보를 연결한다. 해시 기반(Hash-Based) 또는 이와 유사한 효율적인 인덱싱을 사용하면 전체 포인트 클라우드를 검색하지 않고도 관련 셀에 거의 직접적으로 접근할 수 있다. 이러한 구성은 새로운 관측값을 증분 방식으로 삽입하고 주변 포인트를 효율적으로 검색하며 멀리 있거나 불필요한 지도 영역을 독립적으로 관리할 수 있기 때문에 온라인 LiDAR 처리에 적합하다.

증분 매핑(Incremental Mapping)은 로봇이 이동함에 따라 지도가 지속적으로 변화한다는 것을 의미한다. 현재 상태가 보정된 이후 수용된 LiDAR 포인트를 지도 좌표계로 변환하여 로컬 표현에 삽입한다. 제어되지 않는 지도 증가를 방지하기 위해 중복 측정값을 제거하거나 다운샘플링(Downsampling)할 수 있다. 갱신된 지도는 이후 스캔의 기하학적 기준이 되어 지속적인 예측-보정-지도 갱신(Prediction-Correction-Map-Update) 순환 구조를 형성한다.

실시간 오도메트리에서는 무한히 확장되는 전역 포인트 클라우드보다 로컬 지도(Local Map)가 일반적으로 적합하다. 즉각적인 스캔 대 지도(Scan-to-Map) 제약에는 로봇 주변의 기하 구조만 필요하기 때문이다. 플랫폼이 이동하면 새로운 공간 영역이 활성 지도에 추가되고 멀어진 영역은 제거하거나 보관하거나 별도로 처리할 수 있다. 활성 지도의 범위를 제한하면 장거리 주행에서도 메모리 사용량과 최근접 이웃 탐색 성능을 예측 가능한 수준으로 유지할 수 있다.

Faster-LIO는 기본적으로 오도메트리와 로컬 매핑 시스템이므로 장기적인 전역 일관성(Global Consistency)은 단기 상태 추정 정확도와 구분해야 한다. 증분 LiDAR-관성 추정은 드리프트를 크게 감소시킬 수 있지만 임의로 긴 운용 시간에 대해 완전히 드리프트가 없는 전역 궤적을 보장하지는 않는다. 전역적으로 일관된 지도가 필요한 경우 상위 계층에 루프 폐쇄(Loop Closure), GNSS 제약, 포즈 그래프 최적화(Pose-Graph Optimization) 등의 전역 보정 기능을 추가할 수 있다.

LiDAR와 IMU 사이의 긴밀한 결합은 급격한 운동(Aggressive Motion)에서 중요한 장점을 제공한다. 연속적인 LiDAR 스캔 사이의 변위가 큰 경우 IMU 전파가 고주파 자세 예측을 제공하며 LiDAR 기하 구조는 관성 적분 오차가 무제한으로 증가하는 것을 억제한다. 또한 상태 전파에 사용하는 동일한 운동 추정값으로 LiDAR 데이터 획득 구간 전체의 센서 움직임을 표현할 수 있기 때문에 디스큐잉 성능도 향상된다.

그러나 LiDAR-관성 추정은 여전히 관측 가능성(Observability)에 영향을 받는다. 긴 복도, 평평한 도로, 터널, 대형 벽면, 개방된 환경에서는 특정 방향에 대한 기하학적 제약이 약할 수 있다. 이러한 상황에서는 일부 상태 변수를 LiDAR 기하 구조만으로 정확하게 추정하기 어렵다. IMU 정보는 단기 연속성을 유지하는 데 도움을 주지만 장기적으로 강한 제약을 제공하려면 GNSS, 휠 오도메트리 또는 추가적인 인지 센서가 필요할 수 있다.

IMU 초기화(IMU Initialization) 역시 중요한 단계이다. 중력 방향, 초기 자세, 속도, 센서 바이어스가 이후 상태 전파에 영향을 주기 때문이다. 플랫폼이 정지한 상태에서 초기화를 수행하면 일반적으로 중력과 바이어스 관련 상태를 보다 안정적으로 추정할 수 있다. 이동 중 초기화를 수행해야 하는 시스템에서는 더욱 정교한 관측 가능성 처리가 필요하다. 부정확한 초기화는 잘못된 디스큐잉, 일관되지 않은 지도 기하 구조, 느리거나 불안정한 수렴을 발생시킬 수 있다.

동적 객체 역시 기하 잔차가 정적인 환경에서 발생한다는 가정을 위반한다. 차량, 보행자, 지게차, 식생, 이동하는 기계는 이후 일관성을 잃는 지도 포인트를 생성할 수 있다. 잔차 임계값(Residual Threshold), 이웃 검사(Neighborhood Check), 시간적 필터링(Temporal Filtering), 의미론적 처리(Semantic Processing), 지도 노화 전략(Map-Aging Strategy)을 이용하여 영향을 줄일 수 있다. 따라서 산업 환경 적용에서는 추정 정확도뿐만 아니라 로컬 기하 지도의 정보 유지 정책도 고려해야 한다.

계산 효율성(Computational Efficiency)은 IMU 전파, 디스큐잉, 이웃 탐색, 잔차 생성, 반복 상태 갱신, 지도 삽입을 모두 LiDAR 프레임 주기 내에서 수행해야 하기 때문에 매우 중요하다. Faster-LIO는 비용이 큰 공간 연산을 줄이고 증분 방식으로 갱신되는 지도를 유지함으로써 이러한 문제를 해결하는 것을 목표로 한다. 따라서 모든 단계에서 계산량이 큰 전역 정합(Global Registration)에 의존하지 않고도 효율적인 CPU 구현을 통해 고주파 상태 추정을 지원할 수 있다.

파라미터 설정(Parameter Selection)은 정확도와 실시간 성능 모두에 영향을 준다. 복셀 크기(Voxel Size)는 지도 밀도와 기하학적 세부 수준을 결정하고 이웃 설정(Neighborhood Setting)은 평면 추정에 영향을 주며 잔차 임계값은 대응점 제거를 제어한다. 로컬 지도 크기는 메모리와 탐색 비용에 영향을 준다. 지나치게 조밀한 지도는 관측 가능성을 개선하지 않으면서 계산량만 증가시킬 수 있으며 지나치게 거친 표현은 정확한 기하 보정에 필요한 구조를 제거할 수 있다.

센서 특성(Sensor Characteristics) 역시 설정에 반영해야 한다. 높은 채널 수를 갖는 회전형 LiDAR는 조밀한 관측값을 제공하지만 상당한 처리 부하를 발생시킬 수 있으며 솔리드 스테이트(Solid-State) 또는 비반복 스캐닝(Non-Repetitive Scanning) 센서는 서로 다른 공간 샘플링 패턴을 생성한다. IMU 노이즈 밀도(Noise Density), 바이어스 안정성(Bias Stability), 측정 주파수, LiDAR 거리 정확도, 시야각(Field of View)은 모두 추정기의 동작에 영향을 준다. 따라서 실제 센서 구성과 예상 차량 동역학을 이용하여 파라미터를 검증해야 한다.

Faster-LIO에서 생성되는 상태 추정값은 로컬 내비게이션(Local Navigation), 장애물 지도 작성(Obstacle Mapping), 모션 플래닝(Motion Planning), 센서 좌표계 변환(Sensor-Frame Transformation) 등의 하위 자율 기능을 지원할 수 있다. 특히 고주파 자세 출력은 비동기적으로 획득되는 여러 인지 데이터를 공통 월드 좌표계로 변환해야 하는 경우 유용하다. 그러나 안전 중요 제어(Safety-Critical Control)에서는 모든 출력 자세가 동일하게 신뢰할 수 있다고 가정하기보다 추정기 상태, 타임스탬프 유효성, 혁신량(Innovation Magnitude), 센서 가용성을 함께 감시해야 한다.

야외 자율이동로봇(Outdoor AMR)의 거친 지형(Rough Terrain)에서는 병진, 롤(Roll), 피치(Pitch), 진동, 수직 가속도의 급격한 변화가 동시에 발생한다. 이러한 조건에서는 평면 오도메트리(Planar Odometry) 가정이 적합하지 않으며 완전한 3차원 LiDAR-관성 추정(Full 3D LiDAR-Inertial Estimation)의 가치가 높아진다. 기계적 진동 절연(Vibration Isolation), 견고한 센서 장착, 동기화, 보정, 적절한 IMU 품질도 중요하다. 소프트웨어 추정만으로 불안정한 하드웨어 통합이나 손상된 측정값을 완전히 보상할 수는 없기 때문이다.

실용적인 아키텍처에서는 Faster-LIO를 고주파 로컬 상태 추정 계층(High-Rate Local State-Estimation Layer)으로 사용하고 상대적으로 느린 전역 위치 추정 계층(Global Localization Layer)과 연결할 수 있다. 야외에서는 GNSS 또는 RTK가 지리적 기준점을 제공하고 반복적으로 방문하는 영역에서는 루프 폐쇄와 포즈 그래프 최적화가 누적 드리프트를 보정할 수 있다. 로컬 추정기는 부드러운 실시간 운동 추정값을 지속적으로 제공하고 전역 계층은 장기 일관성, 지도 정렬, 임무 수준 좌표계를 관리한다.

따라서 Faster-LIO는 단순히 더 빠른 포인트 클라우드 정합 알고리즘을 의미하는 것이 아니다. 핵심 개념은 관성 전파(Inertial Propagation), LiDAR 모션 보상, 기하 측정 갱신(Geometric Measurement Update), 증분 지도 관리(Incremental Map Management)를 하나의 긴밀 결합형 추정 순환 구조 안에서 효율적으로 통합하는 것이다. 이러한 아키텍처를 통해 로컬 기하 구조와 고주파 관성 운동 정보가 지속적으로 서로를 보정하면서 실시간 로봇 운용에 적합한 수준으로 계산 비용을 유지할 수 있다.

올바르게 적용된 Faster-LIO 프레임워크는 자율주행 차량과 이동 로봇을 위한 고주파 6자유도 위치 추정의 강력한 기반을 제공한다. 실제 성능은 시간 동기화, 보정, 초기화, 센서 품질, 환경 관측 가능성, 지도 파라미터, 계산 자원에 따라 달라진다. 적절한 전역 보정 계층(Global Correction Layer)과 결합하면 증분 LiDAR-관성 오도메트리는 확장 가능한 야외 및 산업용 자율 시스템의 실시간 위치 추정 백본(Real-Time Localization Backbone)을 구성할 수 있다.

##  

## 06.06. Large Scale LiDAR Map Tiling and Loading [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

Large-scale LiDAR mapping requires a different data-management strategy from small indoor SLAM because a continuously accumulated point cloud can eventually exceed practical memory and processing limits. Map tiling divides a large three-dimensional environment into manageable spatial units that can be stored, indexed, loaded, updated, and removed independently while preserving a consistent global coordinate system.

The fundamental concept is spatial partitioning. Instead of representing an entire city, industrial complex, port, campus, mine, or logistics site as one monolithic point cloud, the environment is divided according to predefined geographic regions. Each tile contains LiDAR geometry belonging to a bounded area and carries metadata describing its position, spatial extent, resolution, coordinate frame, version, and relationship to neighboring tiles.

A simple tiling scheme divides the horizontal map plane into regular square cells using global x and y coordinates. A point can be assigned to a tile by quantizing its position according to the selected tile dimensions. Three-dimensional partitioning may additionally divide the vertical axis when structures contain tunnels, bridges, underground spaces, or multiple floors. The appropriate strategy depends on environmental scale and expected robot motion.

Tile size introduces an important engineering tradeoff. Large tiles reduce the number of files and indexing operations but require more memory and loading time whenever only a small portion of the environment is needed. Small tiles support fine-grained streaming and efficient local access but increase metadata, file-system, and management overhead. Tile dimensions should therefore be selected according to map density, storage architecture, localization range, and vehicle speed.

The global coordinate frame provides the common spatial reference that makes independently stored tiles behave as one map. Outdoor systems may use a local Cartesian frame derived from GNSS coordinates or a projected geographic coordinate system. Large coordinate values should be handled carefully because floating-point precision can degrade geometric calculations. Local tile coordinates can be used internally while tile origins preserve their relationship to the global map.

Each tile can contain raw points, downsampled point clouds, surfels, voxels, NDT distributions, occupancy information, or multiple map layers. Localization does not necessarily require the same representation used for visualization or archival storage. A system may retain a dense master map offline while generating lighter localization tiles containing only stable geometric features required for real-time scan matching.

Map generation typically begins by transforming registered LiDAR scans into a globally consistent reference frame. Points are assigned to spatial tiles according to their coordinates and accumulated until each tile is finalized or updated. Voxel downsampling can remove redundant measurements and control density. Statistical filtering, dynamic-object removal, ground classification, and intensity processing may also be applied before the map becomes a localization resource.

Tile boundaries require special consideration because LiDAR registration often uses geometry surrounding the robot rather than points located strictly inside one cell. If the vehicle approaches an edge, loading only its current tile can remove useful surfaces immediately across the boundary. Systems therefore load neighboring tiles or store overlapping margins so that the active local map remains geometrically continuous during transitions between spatial partitions.

The active map is the subset of tiles currently required for localization and navigation. The robot position determines a region of interest, and the map manager loads tiles intersecting that region. Tiles that move outside the active range can be released from memory. This dynamic loading and unloading process allows the total stored map to grow far beyond available RAM while keeping real-time computation bounded by local environmental complexity.

A radius-based loading policy selects all tiles within a specified distance from the robot, while grid-neighborhood policies load the current tile and a fixed number of adjacent cells. More advanced strategies consider vehicle heading, planned route, speed, sensor range, or predicted future poses. A fast-moving outdoor robot can preload tiles ahead of its trajectory so that map data is available before the localization front end requires it.

Asynchronous loading is important because storage latency should not block real-time state estimation. A dedicated map-management thread can monitor the predicted robot position, request required tiles, and populate a cache while the localization process continues using the current active map. Double buffering or staged map replacement can ensure that incomplete loading does not expose the registration algorithm to partially constructed reference geometry.

Caching further reduces storage access. Recently used tiles can remain in memory even after they leave the immediate localization region, allowing rapid reuse if the robot reverses direction or revisits a nearby area. Cache policies may consider recency, distance, expected route, memory pressure, and tile size. The objective is to minimize repeated disk or network transfers while maintaining a predictable upper bound on memory consumption.

Storage performance becomes significant when maps contain billions of points. Individual text-based point-cloud files are generally inefficient for large-scale streaming because parsing and storage overhead become substantial. Binary point-cloud formats, compressed tile packages, memory-mapped structures, spatial databases, or custom voxel representations can provide faster access. The storage design should support partial retrieval rather than requiring the entire global map to be decoded.

A hierarchical spatial index can improve scalability. At the highest level, coarse regions identify which part of the environment contains the robot, while lower levels provide increasingly detailed tiles. Quadtree structures partition two-dimensional space recursively, whereas octrees extend the concept to three dimensions. Hierarchical representations also support level-of-detail selection, allowing distant geometry to remain coarse while nearby localization regions use higher resolution.

Level of detail is valuable when the same map supports multiple consumers. A mission planner may require only coarse road or free-space information, while LiDAR localization needs detailed surfaces around the vehicle. Visualization may use another resolution entirely. Maintaining several representations prevents every subsystem from processing maximum-density point clouds and allows bandwidth, memory, and computation to be allocated according to actual task requirements.

Map tiling must also preserve sufficient overlap for scan-to-map registration. The active reference should normally extend beyond the LiDAR sensing region required by the registration algorithm. If the local map is too small, correspondence quality can degrade near its boundary and produce unstable pose estimates. Loading distance should therefore account for sensor range, expected pose uncertainty, motion between updates, and the geometric neighborhood required by the registration method.

Localization itself can be organized in two stages. A coarse global or regional localization process first determines which map region contains the robot, after which high-resolution tiles are loaded for precise scan matching. This architecture is useful when the initial pose is uncertain or the robot can start from multiple deployment locations. Place recognition, GNSS, map descriptors, or low-resolution registration can provide the initial region hypothesis.

Large maps require explicit version management because physical environments and map-processing algorithms change over time. Each tile can carry a map version, creation time, sensor source, calibration identifier, and processing configuration. Updating one region should not necessarily require rebuilding the entire map. Tile-level versioning allows construction zones, changed buildings, roads, racks, or equipment layouts to be replaced independently while unaffected regions remain valid.

Incremental updates create additional consistency requirements. A newly generated tile must align correctly with neighboring tiles and the global coordinate system before it replaces an older version. Boundary geometry can be checked for discontinuities, and overlapping regions can be compared statistically. Transactional replacement or atomic file switching prevents localization from reading a mixture of incomplete old and new map data during deployment.

Dynamic and temporary objects should normally be excluded from long-term localization tiles. Parked vehicles, pedestrians, movable containers, pallets, vegetation, and construction equipment can otherwise become misleading reference geometry. Multi-session mapping, temporal persistence analysis, semantic filtering, and occupancy statistics can identify structures that remain stable across observations. Stable-map extraction is especially important for industrial sites where scene configuration changes frequently.

Map quality metadata can help localization select reliable geometry. Individual tiles may store point density, timestamp, registration uncertainty, feature richness, dynamic probability, or expected localization quality. If a region contains sparse or repetitive geometry, the state estimator can reduce confidence in LiDAR matching and rely more strongly on IMU, wheel odometry, GNSS, or other sensors until stronger map constraints become available.

Network-based map distribution extends tiling to fleets of robots. A central server can maintain the authoritative global map while each robot downloads only tiles required for its mission. Frequently used regions may be cached on the robot, and changed tiles can be synchronized using version identifiers rather than retransmitting the complete map. This architecture reduces bandwidth and supports coordinated map maintenance across large autonomous fleets.

Failure handling is essential when a required tile cannot be loaded because of corrupted storage, network interruption, incorrect indexing, or map-version mismatch. The localization system should detect missing reference data rather than silently treating an empty map as valid. A fallback mode may continue temporarily using LiDAR-inertial odometry, GNSS, or previously cached geometry while the map manager attempts recovery or requests an alternative data source.

Real-time performance depends on separating map size from active computational workload. A global map may occupy hundreds of gigabytes or more, yet the registration algorithm should process only the geometry needed around the current robot pose. Efficient spatial indexing, bounded active maps, asynchronous streaming, caching, and voxelized representations make computational cost depend primarily on local map density rather than the total mapped area.

For outdoor AMRs, map loading should be coordinated with navigation and vehicle dynamics. Higher speed increases the distance traveled during storage latency and therefore requires earlier prefetching. Route information can predict which tiles will soon be needed, while unexpected detours require rapid neighboring-tile access. The map manager should consequently operate as part of the localization infrastructure rather than as a passive file-loading utility.

A scalable architecture separates the global map repository, spatial index, tile cache, active localization map, and registration front end. The repository stores persistent map products, the index resolves positions to tile identifiers, the cache manages recently accessed data, and the active-map layer assembles nearby geometry. The registration module then interacts with a bounded local representation without needing to understand the total size of the global dataset.

Large-scale LiDAR map tiling and loading is therefore fundamentally a spatial data-management problem tightly connected to localization. Its purpose is not merely to split a large point-cloud file into smaller files, but to ensure that the correct geometric information is available at the correct place and time. With appropriate partitioning, indexing, caching, prefetching, versioning, and quality control, extremely large maps can support reliable real-time autonomous operation.

대규모 LiDAR 매핑(Large-Scale LiDAR Mapping)은 지속적으로 누적되는 포인트 클라우드(Point Cloud)가 결국 실용적인 메모리와 처리 한계를 초과할 수 있기 때문에 소규모 실내 SLAM과는 다른 데이터 관리 전략(Data-Management Strategy)이 필요하다. 지도 타일링(Map Tiling)은 대규모 3차원 환경을 독립적으로 저장, 인덱싱(Indexing), 로딩(Loading), 갱신, 제거할 수 있는 관리 가능한 공간 단위로 분할하면서 일관된 전역 좌표계(Global Coordinate System)를 유지하는 방법이다.

기본 개념은 공간 분할(Spatial Partitioning)이다. 도시, 산업단지, 항만, 캠퍼스, 광산 또는 물류 현장 전체를 하나의 거대한 포인트 클라우드로 표현하는 대신 환경을 미리 정의된 지리적 영역(Geographic Region)에 따라 분할한다. 각각의 타일(Tile)은 제한된 영역에 속하는 LiDAR 기하 정보를 포함하며 위치, 공간 범위(Spatial Extent), 해상도, 좌표계, 버전, 인접 타일과의 관계를 설명하는 메타데이터(Metadata)를 함께 관리한다.

단순한 타일링 방식은 전역 x 및 y 좌표를 이용하여 수평 지도 평면을 일정한 크기의 정사각형 셀로 분할한다. 포인트의 위치를 선택된 타일 크기에 따라 양자화(Quantization)하면 해당 포인트가 속할 타일을 결정할 수 있다. 터널, 교량, 지하 공간 또는 다층 구조가 포함된 환경에서는 수직축까지 추가로 분할하는 3차원 공간 분할(3D Partitioning)을 사용할 수 있다. 적절한 방식은 환경 규모와 예상되는 로봇의 이동 형태에 따라 달라진다.

타일 크기(Tile Size)는 중요한 공학적 상충 관계(Engineering Tradeoff)를 발생시킨다. 큰 타일은 파일 수와 인덱싱 연산을 줄이지만 환경의 작은 일부만 필요한 경우에도 더 많은 메모리와 로딩 시간이 필요하다. 작은 타일은 세밀한 스트리밍(Fine-Grained Streaming)과 효율적인 로컬 접근을 지원하지만 메타데이터, 파일 시스템, 관리 오버헤드(Management Overhead)를 증가시킨다. 따라서 타일 크기는 지도 밀도, 저장장치 구조, 위치 추정 범위, 차량 속도를 고려하여 결정해야 한다.

전역 좌표계(Global Coordinate Frame)는 독립적으로 저장된 타일들이 하나의 지도처럼 동작하도록 만드는 공통 공간 기준을 제공한다. 야외 시스템에서는 GNSS 좌표에서 파생된 로컬 직교 좌표계(Local Cartesian Frame) 또는 투영 지리 좌표계(Projected Geographic Coordinate System)를 사용할 수 있다. 매우 큰 좌표값은 부동소수점 정밀도(Floating-Point Precision)를 저하시킬 수 있으므로 주의해야 한다. 타일 내부에서는 로컬 좌표(Local Tile Coordinate)를 사용하고 타일 원점(Tile Origin)을 통해 전역 지도와의 관계를 유지할 수 있다.

각 타일에는 원시 포인트(Raw Point), 다운샘플링된 포인트 클라우드, 서펠(Surfel), 복셀(Voxel), NDT 분포(NDT Distribution), 점유 정보(Occupancy Information) 또는 여러 지도 계층(Map Layer)을 저장할 수 있다. 위치 추정(Localization)에 반드시 시각화나 보관용 지도와 동일한 표현이 필요한 것은 아니다. 시스템은 오프라인에서 고밀도 마스터 지도(Dense Master Map)를 유지하면서 실시간 스캔 정합에 필요한 안정적인 기하 특징만 포함하는 가벼운 위치 추정용 타일(Localization Tile)을 별도로 생성할 수 있다.

지도 생성(Map Generation)은 일반적으로 정합된 LiDAR 스캔을 전역적으로 일관된 기준 좌표계로 변환하는 과정에서 시작한다. 포인트는 좌표에 따라 공간 타일에 할당되고 각 타일이 완성되거나 갱신될 때까지 누적된다. 복셀 다운샘플링(Voxel Downsampling)을 통해 중복 측정값을 제거하고 포인트 밀도를 제어할 수 있다. 지도 자체를 위치 추정 자원으로 사용하기 전에 통계적 필터링(Statistical Filtering), 동적 객체 제거(Dynamic-Object Removal), 지면 분류(Ground Classification), 반사 강도 처리(Intensity Processing) 등을 적용할 수도 있다.

LiDAR 정합은 일반적으로 하나의 셀 내부에 엄격하게 포함된 포인트만 사용하는 것이 아니라 로봇 주변의 기하 구조를 사용하기 때문에 타일 경계(Tile Boundary)를 특별히 고려해야 한다. 차량이 타일 경계에 접근했을 때 현재 타일만 로딩하면 경계 바로 반대편에 존재하는 유용한 표면 정보가 사라질 수 있다. 따라서 시스템은 인접 타일(Neighboring Tile)을 함께 로딩하거나 중첩 여유 영역(Overlapping Margin)을 저장하여 공간 분할 사이를 이동할 때 활성 로컬 지도(Active Local Map)의 기하학적 연속성을 유지한다.

활성 지도(Active Map)는 현재 위치 추정과 내비게이션에 필요한 타일의 부분집합이다. 로봇 위치에 따라 관심 영역(Region of Interest)을 결정하고 지도 관리자(Map Manager)가 해당 영역과 교차하는 타일을 로딩한다. 활성 범위를 벗어난 타일은 메모리에서 제거할 수 있다. 이러한 동적 로딩 및 언로딩(Dynamic Loading and Unloading)을 사용하면 전체 저장 지도의 크기가 사용 가능한 RAM을 크게 초과하더라도 실시간 계산량은 로컬 환경의 복잡도에 따라 제한할 수 있다.

반경 기반 로딩 정책(Radius-Based Loading Policy)은 로봇으로부터 지정된 거리 안에 있는 모든 타일을 선택하며 그리드 이웃 정책(Grid-Neighborhood Policy)은 현재 타일과 일정 개수의 인접 셀을 로딩한다. 보다 발전된 전략에서는 차량 진행 방향, 계획 경로(Planned Route), 속도, 센서 범위, 예측된 미래 자세를 고려할 수 있다. 고속으로 이동하는 야외 로봇은 이동 궤적 전방의 타일을 미리 로딩(Prefetching)하여 위치 추정 프론트엔드가 필요로 하기 전에 지도 데이터를 준비할 수 있다.

저장장치 지연(Storage Latency)이 실시간 상태 추정을 차단해서는 안 되기 때문에 비동기 로딩(Asynchronous Loading)이 중요하다. 전용 지도 관리 스레드(Map-Management Thread)는 예측된 로봇 위치를 감시하고 필요한 타일을 요청하여 캐시(Cache)를 채우는 동안 위치 추정 프로세스는 현재 활성 지도를 계속 사용할 수 있다. 이중 버퍼링(Double Buffering) 또는 단계적 지도 교체(Staged Map Replacement)를 사용하면 로딩이 완료되지 않은 상태의 기준 기하 구조가 정합 알고리즘에 노출되는 것을 방지할 수 있다.

캐싱(Caching)은 저장장치 접근을 추가로 감소시킨다. 최근에 사용한 타일을 즉각적인 위치 추정 범위에서 벗어난 이후에도 메모리에 유지하면 로봇이 방향을 바꾸거나 인접 영역을 다시 방문할 때 빠르게 재사용할 수 있다. 캐시 정책(Cache Policy)은 최근 사용 시점, 거리, 예상 경로, 메모리 사용 압력(Memory Pressure), 타일 크기 등을 고려할 수 있다. 목표는 메모리 소비의 상한을 예측 가능한 수준으로 유지하면서 반복적인 디스크 또는 네트워크 전송을 최소화하는 것이다.

지도에 수십억 개의 포인트가 포함되면 저장 성능(Storage Performance)이 중요해진다. 개별 텍스트 기반 포인트 클라우드 파일은 파싱(Parsing)과 저장 오버헤드가 커지기 때문에 대규모 스트리밍에 일반적으로 비효율적이다. 바이너리 포인트 클라우드 형식(Binary Point-Cloud Format), 압축 타일 패키지(Compressed Tile Package), 메모리 매핑 구조(Memory-Mapped Structure), 공간 데이터베이스(Spatial Database), 사용자 정의 복셀 표현 등을 사용하면 더 빠른 접근이 가능하다. 저장 구조는 전체 전역 지도를 디코딩하지 않고 필요한 일부 데이터만 검색할 수 있어야 한다.

계층형 공간 인덱스(Hierarchical Spatial Index)를 사용하면 확장성을 향상시킬 수 있다. 최상위 계층에서는 거친 영역(Coarse Region)을 이용하여 로봇이 위치한 환경 부분을 식별하고 하위 계층에서는 점차 세밀한 타일을 제공한다. 쿼드트리(Quadtree)는 2차원 공간을 재귀적으로 분할하고 옥트리(Octree)는 이를 3차원으로 확장한다. 계층형 표현은 세부 수준(Level of Detail) 선택도 지원하여 먼 영역의 기하 정보는 거칠게 유지하고 위치 추정에 필요한 가까운 영역에는 높은 해상도를 사용할 수 있다.

세부 수준(Level of Detail)은 동일한 지도를 여러 시스템이 사용하는 경우 유용하다. 임무 계획기(Mission Planner)는 거친 도로 또는 자유 공간 정보만 필요할 수 있지만 LiDAR 위치 추정에는 차량 주변의 세밀한 표면 정보가 필요하다. 시각화에는 또 다른 해상도를 사용할 수 있다. 여러 표현을 유지하면 모든 서브시스템이 최대 밀도의 포인트 클라우드를 처리할 필요가 없으며 실제 작업 요구에 따라 대역폭, 메모리, 계산 자원을 할당할 수 있다.

지도 타일링은 스캔 대 지도 정합(Scan-to-Map Registration)에 충분한 중첩 영역(Overlap)을 유지해야 한다. 활성 기준 지도는 일반적으로 정합 알고리즘에 필요한 LiDAR 감지 영역보다 넓게 확장되어야 한다. 로컬 지도가 지나치게 작으면 경계 부근에서 대응점 품질(Correspondence Quality)이 저하되어 자세 추정이 불안정해질 수 있다. 따라서 로딩 거리는 센서 범위, 예상 자세 불확실성(Pose Uncertainty), 갱신 사이의 이동량, 정합 방법에 필요한 기하학적 이웃 범위를 고려하여 설정해야 한다.

위치 추정(Localization)은 두 단계로 구성할 수도 있다. 먼저 거친 전역 또는 지역 위치 추정(Coarse Global or Regional Localization)을 통해 로봇이 어느 지도 영역에 존재하는지 결정한 다음 정밀한 스캔 정합을 위해 고해상도 타일을 로딩한다. 이러한 아키텍처는 초기 자세가 불확실하거나 로봇이 여러 배치 위치에서 시작할 수 있는 경우 유용하다. 장소 인식(Place Recognition), GNSS, 지도 디스크립터(Map Descriptor), 저해상도 정합 등을 이용하여 초기 영역 가설(Initial Region Hypothesis)을 생성할 수 있다.

물리적 환경과 지도 처리 알고리즘은 시간에 따라 변화하기 때문에 대규모 지도에서는 명시적인 버전 관리(Version Management)가 필요하다. 각 타일은 지도 버전, 생성 시간, 센서 출처, 보정 식별자(Calibration Identifier), 처리 설정(Processing Configuration)을 포함할 수 있다. 특정 영역을 갱신한다고 해서 반드시 전체 지도를 다시 생성할 필요는 없다. 타일 단위 버전 관리(Tile-Level Versioning)를 이용하면 공사 구역, 변경된 건물, 도로, 랙 또는 장비 배치를 독립적으로 교체하면서 영향을 받지 않는 영역은 그대로 유지할 수 있다.

증분 갱신(Incremental Update)은 추가적인 일관성 조건을 필요로 한다. 새롭게 생성된 타일은 기존 버전을 대체하기 전에 인접 타일 및 전역 좌표계와 정확하게 정렬되어야 한다. 경계 기하 구조를 검사하여 불연속성을 확인할 수 있으며 중첩 영역을 통계적으로 비교할 수도 있다. 트랜잭션 방식 교체(Transactional Replacement) 또는 원자적 파일 전환(Atomic File Switching)을 사용하면 배포 과정에서 위치 추정 시스템이 완성되지 않은 기존 지도와 신규 지도 데이터를 혼합하여 읽는 것을 방지할 수 있다.

동적 객체와 임시 객체는 일반적으로 장기 위치 추정 타일에서 제외해야 한다. 주차 차량, 보행자, 이동식 컨테이너, 팔레트, 식생, 건설 장비 등이 기준 기하 구조에 포함되면 이후 위치 추정에 잘못된 정보를 제공할 수 있다. 다중 세션 매핑(Multi-Session Mapping), 시간적 지속성 분석(Temporal Persistence Analysis), 의미론적 필터링(Semantic Filtering), 점유 통계(Occupancy Statistics)를 이용하면 여러 관측에서 안정적으로 유지되는 구조를 식별할 수 있다. 장면 구성이 빈번하게 변경되는 산업 현장에서는 안정 지도 추출(Stable-Map Extraction)이 특히 중요하다.

지도 품질 메타데이터(Map Quality Metadata)를 이용하면 위치 추정 시스템이 신뢰성 높은 기하 구조를 선택할 수 있다. 개별 타일에는 포인트 밀도, 타임스탬프, 정합 불확실성(Registration Uncertainty), 특징 풍부도(Feature Richness), 동적 확률(Dynamic Probability), 예상 위치 추정 품질 등을 저장할 수 있다. 특정 영역의 기하 구조가 희소하거나 반복적인 경우 상태 추정기는 LiDAR 정합의 신뢰도를 낮추고 더 강한 지도 제약을 얻을 때까지 IMU, 휠 오도메트리, GNSS 또는 다른 센서에 더 높은 비중을 둘 수 있다.

네트워크 기반 지도 배포(Network-Based Map Distribution)는 타일링 개념을 로봇 플릿(Robot Fleet)으로 확장한다. 중앙 서버는 기준 전역 지도(Authoritative Global Map)를 관리하고 각 로봇은 임무 수행에 필요한 타일만 다운로드할 수 있다. 자주 사용되는 영역은 로봇에 캐시할 수 있으며 변경된 타일은 전체 지도를 다시 전송하지 않고 버전 식별자(Version Identifier)를 이용하여 동기화할 수 있다. 이러한 구조는 대역폭을 줄이고 대규모 자율 로봇 플릿에서 통합된 지도 유지 관리를 지원한다.

손상된 저장장치, 네트워크 중단, 잘못된 인덱싱 또는 지도 버전 불일치로 필요한 타일을 로딩하지 못할 수 있으므로 장애 처리(Failure Handling)가 필수적이다. 위치 추정 시스템은 빈 지도를 정상적인 기준 지도로 처리하지 않고 필요한 기준 데이터가 누락되었음을 감지해야 한다. 지도 관리자가 복구를 시도하거나 대체 데이터 소스를 요청하는 동안 LiDAR-관성 오도메트리(LiDAR-Inertial Odometry), GNSS 또는 이전에 캐시된 기하 정보를 이용하여 임시로 동작하는 대체 모드(Fallback Mode)를 사용할 수 있다.

실시간 성능(Real-Time Performance)은 전체 지도 크기와 현재 계산 부하를 분리함으로써 확보할 수 있다. 전역 지도는 수백 기가바이트 또는 그 이상일 수 있지만 정합 알고리즘은 현재 로봇 자세 주변에서 필요한 기하 정보만 처리해야 한다. 효율적인 공간 인덱싱, 제한된 활성 지도(Bounded Active Map), 비동기 스트리밍(Asynchronous Streaming), 캐싱, 복셀화 표현을 이용하면 계산 비용이 전체 지도 면적이 아니라 주로 로컬 지도 밀도에 따라 결정되도록 만들 수 있다.

야외 자율이동로봇(Outdoor AMR)의 경우 지도 로딩은 내비게이션과 차량 동역학(Vehicle Dynamics)을 고려하여 조정해야 한다. 속도가 높아지면 저장장치 지연 시간 동안 이동하는 거리가 증가하므로 더 이른 시점의 프리페칭(Prefetching)이 필요하다. 경로 정보(Route Information)를 이용하면 곧 필요할 타일을 예측할 수 있으며 예상하지 못한 우회 상황에서는 주변 타일에 빠르게 접근할 수 있어야 한다. 따라서 지도 관리자는 단순한 파일 로딩 도구가 아니라 위치 추정 인프라(Localization Infrastructure)의 일부로 동작해야 한다.

확장 가능한 아키텍처(Scalable Architecture)는 전역 지도 저장소(Global Map Repository), 공간 인덱스, 타일 캐시(Tile Cache), 활성 위치 추정 지도(Active Localization Map), 정합 프론트엔드(Registration Front End)를 분리한다. 저장소는 영구적인 지도 데이터를 보관하고 인덱스는 위치를 타일 식별자와 연결하며 캐시는 최근 접근한 데이터를 관리한다. 활성 지도 계층은 주변의 기하 정보를 구성하고 정합 모듈은 전체 전역 데이터셋의 크기를 직접 처리할 필요 없이 제한된 로컬 표현과 상호작용한다.

따라서 대규모 LiDAR 지도 타일링 및 로딩(Large-Scale LiDAR Map Tiling and Loading)은 위치 추정과 긴밀하게 연결된 공간 데이터 관리(Spatial Data Management) 문제이다. 목적은 단순히 하나의 거대한 포인트 클라우드 파일을 작은 파일로 나누는 것이 아니라 필요한 기하 정보를 정확한 장소와 시간에 제공하는 것이다. 적절한 공간 분할, 인덱싱, 캐싱, 프리페칭, 버전 관리, 품질 제어를 결합하면 매우 큰 지도에서도 신뢰성 높은 실시간 자율 운용을 지원할 수 있다.

##  

## 06.07. LiDAR SLAM Robustness Degenerate Environments Tunnels

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

LiDAR SLAM robustness becomes particularly challenging in degenerate environments where the surrounding geometry does not provide sufficient independent constraints for estimating all six degrees of freedom. Tunnels are a representative case because long parallel walls, repetitive surfaces, limited lateral variation, and extended straight sections can make different robot poses produce very similar LiDAR observations.

Geometric degeneracy occurs when scan registration strongly constrains some motion directions while leaving others weakly observable. A tunnel wall may accurately constrain displacement perpendicular to its surface but provide little information about translation along the tunnel axis. Similarly, a flat road surface strongly constrains vertical position and attitude while contributing limited information about longitudinal motion. The resulting optimization problem becomes poorly conditioned.

This condition can be examined mathematically through the Hessian or information matrix generated during scan matching. Large eigenvalues correspond to directions with strong geometric constraints, whereas very small eigenvalues indicate weakly observable directions. The ratio between dominant and weak eigenvalues provides an indication of conditioning. Detecting these patterns allows the SLAM system to recognize that a numerically converged registration may still contain substantial uncertainty.

Degeneracy should therefore be treated as an estimation-quality problem rather than simply as a registration failure. ICP, NDT, or LiDAR-inertial registration may report convergence even when the environment cannot uniquely determine the complete pose. Accepting such a solution with excessive confidence can inject incorrect constraints into odometry or a pose graph, causing drift, map deformation, and unstable corrections later in the trajectory.

Long tunnels present several simultaneous difficulties. Their cross-sectional geometry may remain almost constant over hundreds of meters, structural elements can repeat at regular intervals, and distant surfaces may provide limited angular diversity. Vehicles may also travel primarily along the tunnel axis, repeatedly observing similar geometry. Consequently, longitudinal translation can become weakly constrained even when lateral position, height, roll, and pitch remain accurately estimated.

Repetitive structures create another form of ambiguity. Lighting fixtures, wall panels, columns, ventilation equipment, emergency doors, and construction segments may appear at nearly uniform intervals. Scan matching can align the current observation with the wrong repeated structure while still producing a relatively low residual. This perceptual aliasing can generate discrete position errors rather than the gradual drift normally associated with weak geometric constraints.

Feature selection can improve robustness by emphasizing geometrically informative measurements. Planar points, edges, corners, discontinuities, tunnel entrances, equipment installations, intersections, and irregular structural regions provide different constraint directions. A registration front end can evaluate local curvature, surface normals, spatial distribution, or information contribution and prioritize measurements that improve observability instead of treating every LiDAR return equally.

However, feature-rich processing cannot create information that the physical environment does not contain. If a long tunnel is nearly uniform, additional computation on the same geometry cannot fully recover longitudinal position. A robust system must recognize this limitation and introduce complementary sensing rather than forcing the LiDAR optimizer to estimate an unobservable state. This principle is fundamental to reliable SLAM in structurally degenerate environments.

An IMU provides essential short-term support when LiDAR geometry becomes weak. Gyroscope measurements stabilize orientation changes, while accelerometer integration contributes motion information between scans. In tightly coupled LiDAR-inertial odometry, inertial propagation maintains state continuity and LiDAR updates constrain observable components. Although IMU integration also drifts, it can prevent immediate registration instability while the platform passes through temporarily degenerate regions.

Wheel odometry can provide an additional longitudinal constraint for wheeled robots operating on predictable surfaces. Encoder measurements directly estimate wheel rotation and therefore approximate forward displacement even when tunnel geometry changes very little. Wheel slip, tire deformation, uneven terrain, and skid steering reduce accuracy, so wheel odometry should be modeled with uncertainty rather than treated as an exact measurement.

GNSS can provide powerful global position constraints near tunnel entrances and in open sections but is usually unavailable or unreliable underground. Multipath and signal degradation may also occur before complete signal loss. A robust system should detect GNSS quality changes and smoothly modify measurement weighting instead of abruptly trusting or rejecting the sensor. The last reliable global observation can still provide an important anchor before entering a GNSS-denied region.

Additional infrastructure can be valuable for long or safety-critical tunnels. UWB anchors, reflective landmarks, AprilTag-like visual markers, RFID references, surveyed LiDAR landmarks, magnetic markers, or other artificial features can provide absolute or semi-absolute position observations. These references convert an otherwise repetitive environment into one containing identifiable spatial events and can periodically reset accumulated longitudinal uncertainty.

Multi-modal perception can further reduce dependence on LiDAR geometry. Cameras may observe textures, signs, lane markings, equipment labels, lights, or structural details that are difficult to distinguish geometrically. Radar can provide complementary range and velocity information under dust, darkness, or adverse visibility. Sensor fusion is most effective when modalities contribute genuinely different information rather than redundant measurements affected by the same degeneracy.

Motion models also provide useful constraints. Ground vehicles cannot instantaneously move in arbitrary six-dimensional directions, and their feasible motion is limited by steering geometry, velocity, acceleration, and terrain interaction. Incorporating nonholonomic or vehicle-dynamic constraints can suppress physically implausible pose updates. These models should remain soft constraints because rough terrain, wheel slip, impacts, or unusual maneuvers can violate simplified assumptions.

Adaptive covariance is important when degeneracy is detected. Instead of assigning a single confidence value to the entire LiDAR pose update, uncertainty can be increased specifically along poorly observable directions. For example, lateral and vertical corrections may remain trustworthy while longitudinal translation receives much lower confidence. Direction-dependent uncertainty allows the fusion estimator to preserve useful LiDAR information without over-trusting unsupported motion components.

Robust optimization complements degeneracy handling by reducing the effect of incorrect correspondences and transient objects. Huber, Cauchy, or similar loss functions can suppress unusually large residuals, while correspondence gating removes geometrically inconsistent matches. Robust estimation cannot solve fundamental observability loss, but it prevents outliers from further damaging an already weak optimization problem and improves stability when tunnel traffic or maintenance equipment is present.

Dynamic objects are especially problematic when they occupy a large portion of the sensor field of view. Trucks, trains, construction machinery, pedestrians, or nearby vehicles can generate strong geometric surfaces that move independently of the static tunnel. Temporal consistency checks, semantic filtering, motion segmentation, and persistent-map selection help ensure that localization relies primarily on infrastructure rather than transient objects.

Map-based localization can outperform pure scan-to-scan odometry when a reliable prior tunnel map exists. A local reference map aggregates stable geometry from multiple observations and can include surveyed landmarks that are not visible in a single previous scan. Nevertheless, a highly repetitive map can still contain ambiguous regions. Map tiles should therefore include localization-quality metadata describing feature richness, expected degeneracy, and available absolute references.

Keyframes should be selected with observability as well as distance and rotation in mind. Creating many nearly identical keyframes along a uniform tunnel adds storage and graph complexity without adding substantial information. More valuable keyframes occur near entrances, curves, intersections, structural transitions, equipment rooms, elevation changes, and distinctive landmarks. Information-aware keyframe selection can therefore improve both computational efficiency and global map quality.

Loop closure requires particularly conservative validation in repetitive tunnels. Two physically different tunnel segments may produce highly similar point clouds and descriptors, creating false loop candidates. Geometric registration alone may not always reject them because repeated structures can align convincingly. Temporal consistency, trajectory feasibility, accumulated distance, external landmarks, multi-modal descriptors, and multiple sequential observations should therefore contribute to loop verification.

False loop closures are often more damaging than missed loop closures because a single incorrect global constraint can deform a large portion of the map. Pose-graph systems should apply robust kernels, switchable or rejectable constraints, and post-optimization consistency checks where appropriate. A candidate that conflicts strongly with trusted inertial, odometric, or surveyed information should not be accepted solely because its local point-cloud alignment score appears favorable.

Failure detection should operate continuously rather than only after localization has visibly diverged. Useful indicators include Hessian conditioning, eigenvalue ratios, registration residuals, correspondence counts, innovation magnitude, IMU disagreement, pose jumps, map overlap, and estimated covariance growth. Combining several indicators produces a more reliable estimator-health assessment than using a single scan-matching fitness score.

When localization confidence falls below an operational threshold, the robot should enter a defined degraded mode rather than continue using uncertain poses without distinction. Depending on the application, the system may reduce speed, increase following margins, rely more heavily on inertial and wheel measurements, search for known landmarks, or stop at a safe location. Localization uncertainty should therefore influence planning and safety behavior as well as the SLAM estimator itself.

Tunnel transitions deserve special handling because entering and leaving a degenerate region changes sensor observability rapidly. Before entry, the system can establish a high-confidence global pose using GNSS, mapped landmarks, or other references. During traversal, uncertainty can be propagated explicitly. At the exit, newly available global measurements should correct accumulated drift gradually and consistently rather than generating an abrupt pose discontinuity.

A practical robust architecture combines high-rate LiDAR-inertial odometry, degeneracy detection, direction-dependent covariance, vehicle-motion constraints, optional wheel odometry, and a global correction layer. GNSS, surveyed landmarks, UWB, vision, or other absolute references can be introduced where available. The system should dynamically change sensor weighting according to measured observability and sensor health instead of relying on one fixed fusion configuration.

Robust LiDAR SLAM in tunnels is therefore not achieved by selecting a single superior registration algorithm. Reliability comes from recognizing when environmental geometry is informative, identifying which motion directions have become weakly observable, representing that uncertainty correctly, and introducing independent constraints when necessary. Degeneracy-aware estimation transforms a potentially hidden localization weakness into an explicit state that the autonomous system can manage.

For outdoor and industrial AMRs, this approach extends beyond tunnels to corridors, warehouses, underground facilities, long walls, open roads, mines, and repetitive production environments. The central engineering principle remains the same: geometric registration should be trusted only in the dimensions supported by actual observations. Combining observability analysis, multi-sensor fusion, robust optimization, map intelligence, and controlled degraded operation provides a practical foundation for dependable LiDAR SLAM.

LiDAR SLAM 강인성(Robustness)은 주변 기하 구조가 6자유도(Six Degrees of Freedom) 전체를 추정하는 데 충분한 독립적인 제약을 제공하지 못하는 퇴화 환경(Degenerate Environment)에서 특히 어려워진다. 터널(Tunnel)은 대표적인 사례로, 길게 이어지는 평행 벽면, 반복적인 표면, 제한된 횡방향 변화, 긴 직선 구간으로 인해 서로 다른 로봇 자세가 매우 유사한 LiDAR 관측값을 생성할 수 있다.

기하학적 퇴화(Geometric Degeneracy)는 스캔 정합(Scan Registration)이 일부 운동 방향은 강하게 제약하지만 다른 방향은 약하게 관측되는 경우 발생한다. 터널 벽은 표면에 수직인 방향의 변위를 정확하게 제약할 수 있지만 터널 축 방향의 병진에는 거의 정보를 제공하지 못할 수 있다. 마찬가지로 평탄한 도로 표면은 수직 위치와 자세를 강하게 제약하지만 종방향 운동에는 제한적인 정보만 제공한다. 그 결과 최적화 문제의 조건 상태(Conditioning)가 악화된다.

이러한 상태는 스캔 정합 과정에서 생성되는 헤시안 행렬(Hessian Matrix) 또는 정보 행렬(Information Matrix)을 통해 수학적으로 분석할 수 있다. 큰 고유값(Eigenvalue)은 강한 기하학적 제약을 갖는 방향을 의미하며 매우 작은 고유값은 관측 가능성이 낮은 방향을 나타낸다. 지배적인 고유값과 작은 고유값 사이의 비율은 조건 상태를 판단하는 지표가 될 수 있다. 이러한 패턴을 검출하면 수치적으로 정합이 수렴했더라도 상당한 불확실성이 존재한다는 것을 SLAM 시스템이 인식할 수 있다.

따라서 퇴화는 단순한 정합 실패(Registration Failure)가 아니라 추정 품질 문제(Estimation-Quality Problem)로 처리해야 한다. ICP, NDT 또는 LiDAR-관성 정합(LiDAR-Inertial Registration)이 수렴을 보고하더라도 환경 자체가 전체 자세를 유일하게 결정할 수 없는 경우가 있다. 이러한 결과에 지나치게 높은 신뢰도를 부여하면 잘못된 제약이 오도메트리(Odometry)나 포즈 그래프(Pose Graph)에 입력되어 이후 궤적에서 드리프트, 지도 변형, 불안정한 보정을 발생시킬 수 있다.

긴 터널(Long Tunnel)은 여러 가지 어려움을 동시에 발생시킨다. 터널의 단면 기하 구조는 수백 미터에 걸쳐 거의 동일하게 유지될 수 있고 구조물이 일정한 간격으로 반복되며 먼 표면은 제한된 각도 다양성(Angular Diversity)만 제공할 수 있다. 차량 역시 주로 터널 축 방향으로 이동하면서 유사한 기하 구조를 반복적으로 관측한다. 따라서 횡방향 위치, 높이, 롤(Roll), 피치(Pitch)는 비교적 정확하게 추정되더라도 종방향 병진(Longitudinal Translation)은 약하게 제약될 수 있다.

반복 구조(Repetitive Structure)는 또 다른 형태의 모호성(Ambiguity)을 발생시킨다. 조명 설비, 벽 패널, 기둥, 환기 장비, 비상문, 건설 세그먼트 등이 거의 일정한 간격으로 반복될 수 있다. 스캔 정합은 현재 관측을 잘못된 반복 구조에 정렬하면서도 비교적 낮은 잔차(Residual)를 생성할 수 있다. 이러한 지각적 앨리어싱(Perceptual Aliasing)은 약한 기하학적 제약에서 일반적으로 나타나는 점진적 드리프트와 달리 불연속적인 위치 오차를 발생시킬 수 있다.

특징 선택(Feature Selection)은 기하학적으로 정보량이 높은 측정값을 강조함으로써 강인성을 향상시킬 수 있다. 평면 포인트, 에지(Edge), 코너(Corner), 불연속 영역, 터널 입구, 설비 설치 구역, 교차로, 불규칙한 구조 영역은 서로 다른 방향의 제약을 제공한다. 정합 프론트엔드(Registration Front End)는 국부 곡률(Local Curvature), 표면 법선(Surface Normal), 공간 분포, 정보 기여도(Information Contribution)를 평가하여 모든 LiDAR 반사점을 동일하게 취급하는 대신 관측 가능성을 향상시키는 측정값을 우선적으로 사용할 수 있다.

그러나 특징 기반 처리(Feature-Rich Processing)를 강화하더라도 물리적 환경에 존재하지 않는 정보를 새롭게 만들어낼 수는 없다. 긴 터널이 거의 균일한 구조를 가진다면 동일한 기하 데이터에 추가적인 계산을 수행하더라도 종방향 위치를 완전히 복원할 수 없다. 강인한 시스템은 이러한 한계를 인식하고 LiDAR 최적화기가 관측 불가능한 상태를 억지로 추정하도록 하기보다 상호 보완적인 센싱(Complementary Sensing)을 도입해야 한다. 이는 구조적으로 퇴화된 환경에서 신뢰성 높은 SLAM을 구현하기 위한 기본 원칙이다.

IMU는 LiDAR 기하 구조가 약해지는 경우 필수적인 단기 지원 정보를 제공한다. 자이로스코프(Gyroscope) 측정값은 자세 변화를 안정화하고 가속도계(Accelerometer) 적분은 스캔 사이의 운동 정보를 제공한다. 긴밀 결합형 LiDAR-관성 오도메트리(Tightly Coupled LiDAR-Inertial Odometry)에서는 관성 전파(Inertial Propagation)가 상태 연속성을 유지하고 LiDAR 갱신이 관측 가능한 성분을 제약한다. IMU 적분 역시 드리프트를 발생시키지만 일시적인 퇴화 영역을 통과하는 동안 즉각적인 정합 불안정을 방지할 수 있다.

휠 오도메트리(Wheel Odometry)는 예측 가능한 표면에서 운행하는 바퀴형 로봇에 추가적인 종방향 제약(Longitudinal Constraint)을 제공할 수 있다. 엔코더(Encoder) 측정값은 휠 회전을 직접 측정하므로 터널 기하 구조의 변화가 거의 없는 상황에서도 전방 이동 거리를 근사할 수 있다. 그러나 휠 슬립(Wheel Slip), 타이어 변형, 불규칙한 지형, 스키드 조향(Skid Steering)은 정확도를 저하시킬 수 있으므로 휠 오도메트리를 정확한 측정값으로 간주하기보다 불확실성을 포함하여 모델링해야 한다.

GNSS는 터널 입구 주변과 개방된 구간에서 강력한 전역 위치 제약(Global Position Constraint)을 제공할 수 있지만 지하에서는 일반적으로 사용할 수 없거나 신뢰성이 낮다. 완전히 신호가 손실되기 전에도 다중경로(Multipath)와 신호 품질 저하가 발생할 수 있다. 강인한 시스템은 GNSS를 갑자기 신뢰하거나 배제하기보다 품질 변화를 검출하고 측정 가중치를 부드럽게 조절해야 한다. 마지막으로 신뢰할 수 있었던 전역 관측값은 GNSS 음영 구간(GNSS-Denied Region)에 진입하기 전 중요한 기준점(Anchor)이 될 수 있다.

길거나 안전 중요도가 높은 터널에서는 추가적인 인프라(Infrastructure)가 유용할 수 있다. UWB 앵커(UWB Anchor), 반사형 랜드마크(Reflective Landmark), AprilTag 계열 시각 마커, RFID 기준점, 측량된 LiDAR 랜드마크(Surveyed LiDAR Landmark), 자기 마커(Magnetic Marker) 또는 기타 인공 특징을 이용하여 절대 또는 준절대 위치 관측을 제공할 수 있다. 이러한 기준점은 반복적인 환경에 식별 가능한 공간 이벤트를 추가하고 누적되는 종방향 불확실성을 주기적으로 초기화할 수 있다.

다중 모달 인지(Multi-Modal Perception)는 LiDAR 기하 구조에 대한 의존도를 추가로 줄일 수 있다. 카메라는 기하학적으로 구별하기 어려운 텍스처(Texture), 표지판, 차선 표시, 장비 라벨, 조명, 구조적 세부 요소를 관측할 수 있다. 레이더(Radar)는 먼지, 어둠 또는 열악한 시야 환경에서도 보완적인 거리 및 속도 정보를 제공할 수 있다. 센서 융합(Sensor Fusion)은 여러 센서가 동일한 퇴화 현상의 영향을 받는 중복 측정값이 아니라 실질적으로 서로 다른 정보를 제공할 때 가장 효과적이다.

운동 모델(Motion Model) 역시 유용한 제약을 제공한다. 지상 차량은 임의의 6차원 방향으로 순간적으로 이동할 수 없으며 가능한 운동은 조향 기하 구조(Steering Geometry), 속도, 가속도, 지형과의 상호작용에 의해 제한된다. 비홀로노믹 제약(Nonholonomic Constraint) 또는 차량 동역학 제약(Vehicle-Dynamic Constraint)을 통합하면 물리적으로 불가능한 자세 갱신을 억제할 수 있다. 다만 거친 지형, 휠 슬립, 충격 또는 비정상적인 기동은 단순화된 가정을 위반할 수 있으므로 이러한 모델은 연성 제약(Soft Constraint)으로 적용하는 것이 적절하다.

적응형 공분산(Adaptive Covariance)은 퇴화가 검출되었을 때 중요하다. 전체 LiDAR 자세 갱신에 하나의 신뢰도만 부여하는 대신 관측 가능성이 낮은 방향에 대해서만 불확실성을 증가시킬 수 있다. 예를 들어 횡방향과 수직 방향 보정은 계속 신뢰할 수 있지만 종방향 병진에는 훨씬 낮은 신뢰도를 부여할 수 있다. 방향 의존적 불확실성(Direction-Dependent Uncertainty)을 사용하면 융합 추정기가 지원되지 않는 운동 성분을 과도하게 신뢰하지 않으면서 유용한 LiDAR 정보를 유지할 수 있다.

강인 최적화(Robust Optimization)는 잘못된 대응점과 일시적인 객체의 영향을 감소시켜 퇴화 처리를 보완한다. 후버(Huber), 코시(Cauchy) 또는 이와 유사한 손실 함수(Loss Function)는 비정상적으로 큰 잔차의 영향을 억제하며 대응점 게이팅(Correspondence Gating)은 기하학적으로 일관되지 않은 대응을 제거한다. 강인 추정만으로 근본적인 관측 가능성 손실을 해결할 수는 없지만 이상치가 이미 약한 최적화 문제를 더욱 악화시키는 것을 방지하고 터널 내 교통이나 유지보수 장비가 존재하는 상황에서 안정성을 향상시킨다.

동적 객체(Dynamic Object)가 센서 시야(Field of View)의 상당 부분을 차지하는 경우 특히 문제가 된다. 트럭, 열차, 건설 장비, 보행자 또는 주변 차량은 정적인 터널 구조와 독립적으로 움직이는 강한 기하 표면을 생성할 수 있다. 시간적 일관성 검사(Temporal Consistency Check), 의미론적 필터링(Semantic Filtering), 운동 분할(Motion Segmentation), 지속 지도 선택(Persistent-Map Selection)을 통해 위치 추정이 일시적인 객체보다 고정된 인프라를 중심으로 수행되도록 할 수 있다.

신뢰할 수 있는 사전 터널 지도(Prior Tunnel Map)가 존재하는 경우 지도 기반 위치 추정(Map-Based Localization)은 순수한 스캔 대 스캔 오도메트리(Scan-to-Scan Odometry)보다 우수한 성능을 제공할 수 있다. 로컬 기준 지도(Local Reference Map)는 여러 관측으로부터 안정적인 기하 구조를 집계할 수 있으며 직전 스캔 하나에서는 관측하기 어려운 측량 랜드마크도 포함할 수 있다. 그러나 매우 반복적인 지도는 여전히 모호한 영역을 포함할 수 있으므로 지도 타일(Map Tile)에 특징 풍부도, 예상 퇴화 수준, 사용 가능한 절대 기준점 등을 설명하는 위치 추정 품질 메타데이터(Localization-Quality Metadata)를 포함하는 것이 바람직하다.

키프레임(Keyframe)은 거리와 회전뿐 아니라 관측 가능성도 고려하여 선택해야 한다. 균일한 터널을 따라 거의 동일한 키프레임을 많이 생성하면 상당한 새로운 정보를 추가하지 않으면서 저장 공간과 그래프 복잡도만 증가한다. 터널 입구, 곡선 구간, 교차로, 구조 전환 구간, 설비실, 고도 변화, 특징적인 랜드마크 주변의 키프레임이 더 높은 가치를 갖는다. 따라서 정보 기반 키프레임 선택(Information-Aware Keyframe Selection)은 계산 효율성과 전역 지도 품질을 동시에 향상시킬 수 있다.

반복적인 터널에서는 루프 폐쇄(Loop Closure)를 특히 보수적으로 검증해야 한다. 물리적으로 서로 다른 두 터널 구간이 매우 유사한 포인트 클라우드와 디스크립터(Descriptor)를 생성하여 잘못된 루프 후보(False Loop Candidate)를 만들 수 있다. 반복 구조는 실제로도 높은 정합도를 생성할 수 있으므로 기하학적 정합만으로 항상 이를 제거할 수 있는 것은 아니다. 따라서 시간적 일관성, 궤적의 물리적 타당성, 누적 이동 거리, 외부 랜드마크, 다중 모달 디스크립터(Multi-Modal Descriptor), 연속된 여러 관측을 루프 검증에 함께 활용해야 한다.

잘못된 루프 폐쇄(False Loop Closure)는 하나의 잘못된 전역 제약이 지도의 넓은 영역을 변형시킬 수 있기 때문에 루프 폐쇄를 놓치는 것보다 더 심각한 문제가 될 수 있다. 포즈 그래프 시스템은 필요에 따라 강인 커널(Robust Kernel), 전환 가능 또는 제거 가능한 제약(Switchable or Rejectable Constraint), 최적화 이후 일관성 검사(Post-Optimization Consistency Check)를 적용해야 한다. 신뢰할 수 있는 관성 정보, 오도메트리 또는 측량 정보와 크게 충돌하는 후보는 로컬 포인트 클라우드 정합 점수가 높다는 이유만으로 수용해서는 안 된다.

장애 검출(Failure Detection)은 위치 추정이 명확하게 발산한 이후가 아니라 지속적으로 수행되어야 한다. 유용한 지표에는 헤시안 조건 상태(Hessian Conditioning), 고유값 비율(Eigenvalue Ratio), 정합 잔차, 대응점 수, 혁신량(Innovation Magnitude), IMU와의 불일치, 자세 급변(Pose Jump), 지도 중첩도(Map Overlap), 추정 공분산 증가 등이 포함된다. 여러 지표를 결합하면 하나의 스캔 정합 적합도 점수(Scan-Matching Fitness Score)에만 의존하는 것보다 신뢰성 높은 추정기 상태(Estimator Health)를 판단할 수 있다.

위치 추정 신뢰도가 운용 임계값(Operational Threshold) 아래로 떨어지면 불확실한 자세를 구분 없이 계속 사용하는 대신 로봇이 정의된 성능 저하 모드(Degraded Mode)로 진입해야 한다. 응용 분야에 따라 시스템은 속도를 낮추고 안전 간격을 증가시키며 관성 및 휠 측정값에 더 크게 의존하거나 알려진 랜드마크를 탐색하거나 안전한 위치에 정지할 수 있다. 따라서 위치 추정 불확실성은 SLAM 추정기뿐만 아니라 경로 계획(Planning)과 안전 동작(Safety Behavior)에도 영향을 주어야 한다.

터널 진입 및 이탈 구간(Tunnel Transition)은 센서의 관측 가능성이 빠르게 변화하기 때문에 특별한 처리가 필요하다. 진입 전에는 GNSS, 지도화된 랜드마크 또는 다른 기준 정보를 이용하여 높은 신뢰도의 전역 자세를 확보할 수 있다. 터널을 통과하는 동안에는 불확실성을 명시적으로 전파할 수 있다. 출구에서 새로운 전역 측정값을 다시 사용할 수 있게 되면 누적된 드리프트를 갑작스러운 자세 불연속(Pose Discontinuity) 없이 점진적이고 일관되게 보정해야 한다.

실용적인 강인 아키텍처(Robust Architecture)는 고주파 LiDAR-관성 오도메트리, 퇴화 검출(Degeneracy Detection), 방향 의존적 공분산, 차량 운동 제약, 선택적인 휠 오도메트리, 전역 보정 계층(Global Correction Layer)을 결합한다. 사용 가능한 환경에서는 GNSS, 측량 랜드마크, UWB, 비전(Vision) 또는 기타 절대 기준 정보를 추가할 수 있다. 시스템은 하나의 고정된 융합 설정에 의존하기보다 측정된 관측 가능성과 센서 상태(Sensor Health)에 따라 센서 가중치를 동적으로 변경해야 한다.

따라서 터널에서의 강인한 LiDAR SLAM은 단 하나의 우수한 정합 알고리즘을 선택하는 것만으로 달성할 수 없다. 신뢰성은 환경 기하 구조가 언제 충분한 정보를 제공하는지를 인식하고, 어떤 운동 방향의 관측 가능성이 약해졌는지를 식별하며, 해당 불확실성을 정확하게 표현하고, 필요한 경우 독립적인 제약을 추가하는 것에서 확보된다. 퇴화 인식 추정(Degeneracy-Aware Estimation)은 숨겨질 수 있는 위치 추정 취약성을 자율 시스템이 명시적으로 관리할 수 있는 상태로 전환한다.

야외 및 산업용 자율이동로봇(Outdoor and Industrial AMR)에서 이러한 접근법은 터널뿐 아니라 복도, 창고, 지하 시설, 긴 벽면, 개방 도로, 광산, 반복적인 생산 환경에도 적용된다. 핵심 공학 원칙은 동일하다. 기하학적 정합은 실제 관측이 지원하는 운동 차원에서만 신뢰해야 한다. 관측 가능성 분석(Observability Analysis), 다중 센서 융합(Multi-Sensor Fusion), 강인 최적화, 지도 지능화(Map Intelligence), 제어된 성능 저하 운용(Controlled Degraded Operation)을 결합하면 신뢰성 높은 LiDAR SLAM을 구축하기 위한 실용적인 기반을 마련할 수 있다.

##  

## 06.08. Solid State LiDAR SLAM Adaptation MID 360 [w/Code]

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

Solid-state LiDAR SLAM requires adaptation because its measurement pattern differs substantially from conventional mechanically rotating multi-beam LiDAR. Sensors such as the Livox MID-360 acquire three-dimensional geometry through a compact scanning architecture with a wide field of view and nontraditional sampling behavior. SLAM algorithms must therefore account for point timing, spatial density, motion distortion, and feature distribution rather than assuming uniform rotating scan rings.

The MID-360 is particularly relevant to mobile robots because its compact form factor and broad three-dimensional coverage support installation on AMRs, quadrupeds, autonomous vehicles, and mapping platforms. Unlike traditional spinning LiDARs that generate highly regular horizontal scan patterns, its observations accumulate according to a different scanning trajectory. The resulting point cloud becomes progressively denser as the sensor observes a region over time.

This sampling behavior affects the meaning of a LiDAR frame. A conventional rotating sensor often defines a scan according to one mechanical revolution, whereas solid-state or non-repetitive scanning sensors may accumulate measurements over a selected temporal interval. Frame duration therefore becomes an algorithmic design parameter. Short intervals reduce motion distortion and latency but contain fewer points, while longer intervals increase spatial density at the cost of greater motion during acquisition.

Accurate per-point timestamps are consequently essential. Every point should retain information describing when it was measured relative to the scan reference time. LiDAR-inertial odometry can combine these timestamps with high-rate IMU measurements to estimate sensor motion throughout acquisition. Each point is then transformed toward a common temporal reference, producing a deskewed cloud that more accurately represents the surrounding geometry from one consistent pose.

Motion compensation is especially important on outdoor AMRs traveling over rough terrain. Translation, yaw, roll, and pitch can change continuously while a frame is being accumulated, so treating all points as simultaneous introduces artificial geometric deformation. Walls can appear curved, poles can become broadened, and road surfaces can become inconsistent. IMU-assisted deskewing removes much of this distortion before scan-to-map registration.

Time synchronization between the MID-360, IMU, computing platform, and other sensors directly influences achievable SLAM accuracy. A constant timing offset can resemble a spatial calibration error during motion, while variable latency produces inconsistent residuals. Hardware-supported synchronization is preferable where available. Otherwise, timestamp handling, clock alignment, communication delay, and driver behavior must be characterized and validated under realistic vehicle dynamics.

Extrinsic calibration between the LiDAR and IMU is equally important. The rigid transformation contains both rotation and translation, and small errors can become significant during rapid angular motion. The calibrated transform should correspond to the physical sensor mounting and remain mechanically stable. Flexible brackets, vibration, or sensor movement can invalidate an otherwise accurate calibration and produce systematic errors that software optimization cannot fully eliminate.

The spatial sampling pattern also changes feature extraction. Algorithms originally designed around fixed LiDAR rings may depend on ordered scan lines, neighboring beam indices, or curvature calculated along a predictable rotational sequence. Such assumptions may not transfer directly to MID-360 data. Feature extraction should instead rely on local three-dimensional neighborhoods, voxel statistics, surface fitting, temporal accumulation, or sensor-specific point organization.

Direct scan-to-map approaches are therefore attractive for solid-state LiDAR. Rather than requiring classical edge and planar features extracted from organized rings, the estimator can transform selected points into the map and construct geometric residuals against nearby surfaces. Point-to-plane constraints are especially useful when local neighborhoods contain reliable planar geometry. This approach integrates naturally with tightly coupled LiDAR-inertial odometry.

Voxel-based maps provide an efficient representation for irregularly sampled point clouds. Space is divided into three-dimensional cells, allowing measurements acquired at different times and directions to contribute to the same local geometric structure. Voxel hashing or similar indexing supports rapid insertion and neighborhood queries without requiring a regular scan topology. It also helps control map density as repeated observations progressively fill previously sparse regions.

The progressive densification characteristic of nontraditional scanning creates both opportunities and challenges. A stationary or slowly moving sensor can observe a region repeatedly and produce increasingly detailed geometry. However, retaining every measurement generates unnecessary redundancy. Voxel downsampling, representative-point selection, local surfels, or statistical summaries can preserve useful structure while preventing map size and nearest-neighbor search cost from growing continuously.

LiDAR-inertial odometry is particularly well suited to MID-360 SLAM because the IMU provides motion information between irregular spatial observations. High-rate inertial propagation predicts position, velocity, and orientation, while LiDAR geometry corrects accumulated error. Frameworks inspired by tightly coupled approaches such as FAST-LIO or Faster-LIO can therefore provide an appropriate architectural basis when their sensor interface, timing, preprocessing, and map representation are adapted correctly.

Initialization remains critical because LiDAR-inertial estimation requires reasonable estimates of gravity direction, orientation, velocity, and IMU biases. A stationary initialization period can simplify estimation of gravity and sensor bias. If the robot must initialize while moving, the estimator requires sufficient excitation and careful observability handling. Incorrect initialization can cause distorted deskewing and inconsistent geometry before the mapping process has established a reliable local reference.

The MID-360 field of view can provide valuable geometry above, below, and around the robot, depending on installation. Mounting position should therefore be selected according to localization rather than obstacle detection alone. Excessive self-occlusion from the vehicle body reduces useful measurements, while a poor mounting angle may concentrate observations on the ground or skyward structures. Stable structural surfaces should occupy a substantial portion of the usable field of view.

Self-points from the robot body should normally be removed before SLAM processing. Chassis panels, wheels, sensor brackets, manipulators, or payload structures can appear in the LiDAR cloud and move relative to the environment. A calibrated exclusion volume or geometric mask can remove these returns. Robots with moving mechanisms may require configuration-dependent masks or semantic filtering because the self-geometry changes during operation.

Near-field geometry requires careful handling because compact robots can place the LiDAR close to body surfaces, protective covers, or payload equipment. Strong concentrations of nearby points may dominate residual construction even though they contribute little environmental information. Minimum-range filtering, self-masking, spatial weighting, and feature-quality checks can prevent these measurements from overwhelming more useful distant structural constraints.

Ground points provide useful roll, pitch, and height information but can dominate the observation set on outdoor platforms. If most accepted points originate from a locally flat road, longitudinal and yaw constraints may remain weak. A balanced selection strategy should preserve walls, poles, curbs, vegetation boundaries, building structures, and other non-ground geometry. Registration quality depends on directional diversity rather than simply maximizing the number of points.

Degenerate environments remain a concern regardless of sensor type. Tunnels, long corridors, open roads, flat fields, and repetitive industrial spaces can leave particular motion dimensions weakly observable. The broad coverage of the MID-360 may improve the chance of observing useful geometry, but it cannot create constraints absent from the environment. Hessian analysis, covariance estimation, or eigenvalue-based observability measures should still be used to detect weak directions.

Dynamic objects can influence solid-state LiDAR SLAM because progressive sampling may repeatedly observe vehicles, pedestrians, machinery, or moving vegetation. If these points enter the local map, later measurements may become inconsistent. Residual gating, temporal persistence, motion segmentation, semantic filtering, and map-aging mechanisms can reduce contamination. Stable infrastructure should dominate the long-term localization representation whenever possible.

Frame accumulation should adapt to vehicle speed and computational requirements. At low speed, a longer accumulation interval can increase point density without producing severe motion distortion after deskewing. At high speed, shorter intervals reduce latency and limit the distance traveled between updates. An adaptive policy can consider angular rate, translational velocity, point count, and estimator load instead of using one fixed frame duration under every operating condition.

Processing load can be controlled through selective point sampling. The estimator does not necessarily need every LiDAR return to obtain an accurate pose. Voxel sampling, geometric importance, range, surface orientation, and spatial uniformity can determine which measurements participate in optimization. A smaller but geometrically diverse point set can provide stronger constraints than a much larger collection dominated by redundant observations from the same surfaces.

Local-map management is essential for sustained real-time operation. The active map should cover the region required for scan-to-map matching while removing geometry that has moved far behind the robot. Incremental voxel structures allow insertion and deletion without rebuilding the complete point cloud. For long-distance operation, completed regions can be transferred to submaps, tiled storage, or a global mapping layer while the odometry front end retains only nearby geometry.

Solid-state LiDAR odometry should be separated conceptually from global SLAM. High-rate LiDAR-inertial estimation provides smooth local motion, but residual drift remains over long trajectories. Loop closure, GNSS or RTK, surveyed landmarks, and pose-graph optimization can supply global correction. This layered architecture allows the local estimator to remain computationally bounded while slower processes maintain large-scale map consistency.

Loop detection may use keyframe point clouds generated from accumulated MID-360 observations rather than individual short-duration frames. Keyframes provide denser and more distinctive local geometry for place recognition and verification. However, repeated structures can still generate false candidates, so loop constraints require geometric validation and consistency checks. Global optimization should never assume that a descriptor match alone proves spatial identity.

Outdoor deployment introduces environmental factors such as rain, dust, fog, reflective surfaces, direct sunlight, vibration, and temperature variation. Their influence depends on sensor characteristics and operating conditions, so preprocessing should reject obviously unreliable measurements without unnecessarily removing valid geometry. Estimator-health monitoring can detect abnormal point counts, residual growth, timing problems, or sudden changes in registration quality.

A practical MID-360 SLAM pipeline therefore begins with synchronized LiDAR and IMU acquisition, followed by point-level timestamp handling, motion compensation, self-filtering, and spatial sampling. Inertial propagation predicts the state, scan-to-map geometric residuals correct it, and accepted observations update an incremental voxel map. Degeneracy detection and estimator-health checks determine how strongly the resulting pose should be trusted.

For an outdoor AMR, this local pipeline can be connected to wheel odometry, GNSS/RTK, cameras, or radar according to mission requirements. The MID-360 contributes dense three-dimensional environmental geometry, while the IMU maintains high-rate motion continuity and external sensors provide complementary constraints. Sensor weighting should reflect actual measurement quality and observability instead of remaining fixed across all terrain and operating conditions.

Adapting SLAM to the MID-360 is therefore not simply a matter of replacing one LiDAR driver with another. The scanning pattern changes assumptions about frames, neighborhoods, timing, feature extraction, density, and map updates. A robust implementation treats these characteristics as part of the estimator design and combines timestamp-aware preprocessing, tightly coupled inertial estimation, voxel mapping, observability monitoring, and global correction.

When these adaptations are implemented correctly, compact solid-state LiDAR can provide a strong perception and localization foundation for mobile robots. The resulting architecture can exploit wide three-dimensional coverage without depending on conventional scan-ring assumptions, while incremental mapping keeps computation bounded. Combined with reliable calibration, synchronization, sensor fusion, and global SLAM, MID-360-based localization can support scalable indoor, outdoor, and industrial autonomy.

솔리드 스테이트 LiDAR SLAM(Solid-State LiDAR SLAM)은 측정 패턴이 기존의 기계식 회전형 멀티빔 LiDAR(Mechanically Rotating Multi-Beam LiDAR)와 상당히 다르기 때문에 이에 맞는 적응이 필요하다. Livox MID-360과 같은 센서는 넓은 시야각(Field of View)과 비전통적인 샘플링 방식(Nontraditional Sampling Behavior)을 갖는 소형 스캐닝 구조를 통해 3차원 기하 정보를 획득한다. 따라서 SLAM 알고리즘은 균일한 회전 스캔 링(Scan Ring)을 가정하기보다 포인트 시간 정보, 공간 밀도, 모션 왜곡(Motion Distortion), 특징 분포(Feature Distribution)를 고려해야 한다.

MID-360은 소형 폼팩터(Compact Form Factor)와 넓은 3차원 커버리지(3D Coverage)를 제공하기 때문에 자율이동로봇(AMR), 사족보행 로봇(Quadruped), 자율주행 차량, 매핑 플랫폼 등에 적용하기 적합하다. 매우 규칙적인 수평 스캔 패턴을 생성하는 기존 회전형 LiDAR와 달리 MID-360의 관측값은 서로 다른 스캐닝 궤적(Scanning Trajectory)에 따라 누적된다. 따라서 센서가 일정 영역을 지속적으로 관측하면 시간이 지남에 따라 포인트 클라우드(Point Cloud)의 밀도가 점진적으로 증가한다.

이러한 샘플링 방식은 LiDAR 프레임(Frame)의 의미에도 영향을 준다. 기존 회전형 센서는 하나의 기계적 회전을 기준으로 스캔을 정의하는 경우가 많지만 솔리드 스테이트 또는 비반복 스캐닝(Non-Repetitive Scanning) 센서는 선택된 시간 구간 동안 측정값을 누적하여 하나의 프레임을 구성할 수 있다. 따라서 프레임 지속 시간(Frame Duration)은 알고리즘 설계 파라미터가 된다. 짧은 구간은 모션 왜곡과 지연 시간을 줄이지만 포인트 수가 감소하고, 긴 구간은 공간 밀도를 증가시키지만 데이터 획득 중 로봇의 이동량도 증가한다.

따라서 정확한 포인트별 타임스탬프(Per-Point Timestamp)가 필수적이다. 각 포인트는 스캔 기준 시간에 대해 언제 측정되었는지를 나타내는 정보를 유지해야 한다. LiDAR-관성 오도메트리(LiDAR-Inertial Odometry)는 이러한 타임스탬프와 고주파 IMU 측정값을 결합하여 데이터 획득 과정 전체에서 센서의 움직임을 추정할 수 있다. 이후 각각의 포인트를 공통 시간 기준(Common Temporal Reference)으로 변환하여 하나의 일관된 자세에서 주변 기하 구조를 보다 정확하게 표현하는 디스큐잉 포인트 클라우드(Deskewed Point Cloud)를 생성한다.

모션 보상(Motion Compensation)은 거친 지형을 주행하는 야외 자율이동로봇(Outdoor AMR)에서 특히 중요하다. 하나의 프레임이 누적되는 동안 병진, 요(Yaw), 롤(Roll), 피치(Pitch)가 지속적으로 변화할 수 있으므로 모든 포인트가 동시에 측정되었다고 가정하면 인위적인 기하학적 변형이 발생한다. 벽은 휘어진 것처럼 나타날 수 있고 기둥은 넓게 퍼져 보이며 도로 표면의 일관성이 저하될 수 있다. IMU 보조 디스큐잉(IMU-Assisted Deskewing)은 스캔 대 지도 정합(Scan-to-Map Registration) 이전에 이러한 왜곡의 상당 부분을 제거한다.

MID-360, IMU, 컴퓨팅 플랫폼 및 다른 센서 사이의 시간 동기화(Time Synchronization)는 달성 가능한 SLAM 정확도에 직접적인 영향을 미친다. 일정한 시간 오프셋(Timing Offset)은 로봇이 움직이는 동안 공간 보정 오차처럼 나타날 수 있으며 가변적인 지연 시간은 일관되지 않은 잔차(Residual)를 생성한다. 가능한 경우 하드웨어 기반 동기화(Hardware-Supported Synchronization)를 사용하는 것이 바람직하다. 그렇지 않다면 타임스탬프 처리, 클럭 정렬(Clock Alignment), 통신 지연, 드라이버 동작을 실제 차량 동역학 조건에서 분석하고 검증해야 한다.

LiDAR와 IMU 사이의 외부 파라미터 보정(Extrinsic Calibration)도 동일하게 중요하다. 강체 변환(Rigid Transformation)은 회전과 병진을 모두 포함하며 작은 오차도 빠른 각운동 중에는 큰 영향을 줄 수 있다. 보정된 변환은 실제 센서 장착 상태와 일치해야 하며 기계적으로 안정적으로 유지되어야 한다. 유연한 브래킷, 진동 또는 센서의 물리적 이동은 원래 정확했던 보정값을 무효화하고 소프트웨어 최적화만으로 완전히 제거하기 어려운 체계적인 오차를 발생시킬 수 있다.

공간 샘플링 패턴(Spatial Sampling Pattern)의 차이는 특징 추출(Feature Extraction) 방식에도 영향을 준다. 고정된 LiDAR 링을 기준으로 설계된 알고리즘은 정렬된 스캔 라인(Ordered Scan Line), 인접 빔 인덱스(Neighboring Beam Index), 예측 가능한 회전 순서를 따라 계산되는 곡률 등에 의존할 수 있다. 이러한 가정은 MID-360 데이터에 직접 적용하기 어려울 수 있다. 따라서 특징 추출은 국부 3차원 이웃(Local 3D Neighborhood), 복셀 통계(Voxel Statistics), 표면 피팅(Surface Fitting), 시간적 누적 또는 센서 특화 포인트 구성에 기반하는 것이 적합하다.

따라서 직접적인 스캔 대 지도 방식(Direct Scan-to-Map Approach)은 솔리드 스테이트 LiDAR에 적합하다. 정렬된 링 구조에서 고전적인 에지 및 평면 특징을 추출하는 대신 추정기는 선택된 포인트를 지도 좌표계로 변환하고 주변 표면과의 기하 잔차를 구성할 수 있다. 국부 이웃에 신뢰할 수 있는 평면 기하 구조가 존재하는 경우 점 대 평면 제약(Point-to-Plane Constraint)이 특히 유용하다. 이러한 방식은 긴밀 결합형 LiDAR-관성 오도메트리(Tightly Coupled LiDAR-Inertial Odometry)와 자연스럽게 통합된다.

복셀 기반 지도(Voxel-Based Map)는 불규칙하게 샘플링된 포인트 클라우드를 효율적으로 표현할 수 있다. 공간을 3차원 셀로 분할하면 서로 다른 시간과 방향에서 획득된 측정값을 동일한 국부 기하 구조에 통합할 수 있다. 복셀 해싱(Voxel Hashing) 또는 유사한 인덱싱 방법은 규칙적인 스캔 토폴로지(Scan Topology)를 요구하지 않으면서 빠른 포인트 삽입과 이웃 탐색을 지원한다. 또한 반복 관측을 통해 이전에 희소했던 영역의 밀도가 점차 증가할 때 지도 밀도를 제어하는 데 도움을 준다.

비전통적 스캐닝의 점진적 고밀도화(Progressive Densification)는 장점과 문제점을 동시에 제공한다. 정지하거나 저속으로 이동하는 센서는 동일한 영역을 반복적으로 관측하면서 점차 세밀한 기하 구조를 생성할 수 있다. 그러나 모든 측정값을 유지하면 불필요한 중복 데이터가 발생한다. 복셀 다운샘플링(Voxel Downsampling), 대표 포인트 선택(Representative-Point Selection), 로컬 서펠(Local Surfel), 통계적 요약(Statistical Summary)을 사용하면 유용한 구조를 유지하면서 지도 크기와 최근접 이웃 탐색 비용이 지속적으로 증가하는 것을 방지할 수 있다.

LiDAR-관성 오도메트리는 IMU가 불규칙한 공간 관측 사이의 운동 정보를 제공하기 때문에 MID-360 SLAM에 특히 적합하다. 고주파 관성 전파(Inertial Propagation)는 위치, 속도, 자세를 예측하고 LiDAR 기하 구조는 누적된 오차를 보정한다. 따라서 FAST-LIO 또는 Faster-LIO와 같은 긴밀 결합형 접근법에서 영감을 받은 프레임워크는 센서 인터페이스, 시간 처리, 전처리, 지도 표현을 적절하게 조정할 경우 MID-360 기반 시스템의 적합한 아키텍처 기반이 될 수 있다.

LiDAR-관성 추정에서는 중력 방향, 자세, 속도, IMU 바이어스에 대한 적절한 초기 추정값이 필요하므로 초기화(Initialization)가 여전히 중요하다. 정지 상태의 초기화 구간을 사용하면 중력과 센서 바이어스를 보다 간단하게 추정할 수 있다. 로봇이 이동 중에 초기화되어야 한다면 충분한 운동 여기(Motion Excitation)와 세심한 관측 가능성 처리가 필요하다. 부정확한 초기화는 매핑 과정이 신뢰할 수 있는 로컬 기준을 형성하기 전에 잘못된 디스큐잉과 일관되지 않은 기하 구조를 발생시킬 수 있다.

MID-360의 시야각은 장착 방법에 따라 로봇의 위, 아래, 주변에 걸쳐 유용한 기하 정보를 제공할 수 있다. 따라서 장착 위치(Mounting Position)는 장애물 검출뿐만 아니라 위치 추정 관점에서도 선정해야 한다. 차량 본체로 인한 과도한 자체 가림(Self-Occlusion)은 유용한 측정값을 감소시키며 부적절한 장착 각도는 관측값을 지면이나 상부 구조에 지나치게 집중시킬 수 있다. 안정적인 구조 표면이 실제 사용 가능한 시야의 상당 부분을 차지하도록 설계하는 것이 바람직하다.

로봇 본체에서 발생하는 자체 포인트(Self-Point)는 일반적으로 SLAM 처리 전에 제거해야 한다. 섀시 패널, 휠, 센서 브래킷, 매니퓰레이터(Manipulator), 탑재 구조물이 LiDAR 포인트 클라우드에 나타날 수 있으며 일부 구조물은 환경에 대해 상대적으로 움직일 수도 있다. 보정된 제외 영역(Exclusion Volume) 또는 기하학적 마스크(Geometric Mask)를 이용하여 이러한 반사점을 제거할 수 있다. 움직이는 메커니즘을 가진 로봇에서는 자체 기하 구조가 운용 중 변화하므로 구성 상태에 따른 마스크 또는 의미론적 필터링이 필요할 수 있다.

근거리 기하 구조(Near-Field Geometry)도 주의해서 처리해야 한다. 소형 로봇에서는 LiDAR가 차체 표면, 보호 커버 또는 탑재 장비와 가까이 설치될 수 있다. 근거리 포인트가 높은 밀도로 집중되면 실제 환경 위치 추정에 제공하는 정보량이 적음에도 잔차 계산을 지배할 수 있다. 최소 거리 필터링(Minimum-Range Filtering), 자체 마스킹(Self-Masking), 공간 가중화(Spatial Weighting), 특징 품질 검사(Feature-Quality Check)를 통해 이러한 측정값이 더 유용한 원거리 구조 제약을 압도하는 것을 방지할 수 있다.

지면 포인트(Ground Point)는 롤, 피치, 높이 정보를 제공하는 데 유용하지만 야외 플랫폼에서는 전체 관측값을 지나치게 지배할 수 있다. 수용된 포인트 대부분이 국부적으로 평탄한 도로에서 발생하면 종방향 및 요 방향의 제약은 여전히 약할 수 있다. 균형 잡힌 선택 전략(Balanced Selection Strategy)은 벽, 기둥, 연석(Curb), 식생 경계, 건물 구조 및 기타 비지면 기하 정보를 유지해야 한다. 정합 품질은 단순히 포인트 수를 최대화하는 것보다 방향적 다양성(Directional Diversity)에 더 크게 좌우된다.

퇴화 환경(Degenerate Environment)은 센서 종류와 관계없이 여전히 중요한 문제이다. 터널, 긴 복도, 개방 도로, 평탄한 공간, 반복적인 산업 환경에서는 특정 운동 차원의 관측 가능성이 약해질 수 있다. MID-360의 넓은 커버리지는 유용한 기하 구조를 관측할 가능성을 높일 수 있지만 환경에 존재하지 않는 제약을 생성할 수는 없다. 따라서 헤시안 분석(Hessian Analysis), 공분산 추정(Covariance Estimation), 고유값 기반 관측 가능성 측정(Eigenvalue-Based Observability Measure)을 이용하여 약한 방향을 검출해야 한다.

동적 객체(Dynamic Object)는 점진적인 샘플링 과정에서 차량, 보행자, 기계 또는 움직이는 식생을 반복적으로 관측할 수 있기 때문에 솔리드 스테이트 LiDAR SLAM에 영향을 줄 수 있다. 이러한 포인트가 로컬 지도에 입력되면 이후 측정값과 일관성을 잃을 수 있다. 잔차 게이팅(Residual Gating), 시간적 지속성(Temporal Persistence), 운동 분할(Motion Segmentation), 의미론적 필터링(Semantic Filtering), 지도 노화 메커니즘(Map-Aging Mechanism)을 이용하여 오염을 줄일 수 있다. 가능한 경우 장기 위치 추정 표현에서는 안정적인 인프라가 지배적인 기준이 되어야 한다.

프레임 누적(Frame Accumulation)은 차량 속도와 계산 요구사항에 따라 조정해야 한다. 저속에서는 긴 누적 구간을 사용하여 디스큐잉 이후에도 심각한 모션 왜곡 없이 포인트 밀도를 증가시킬 수 있다. 고속에서는 짧은 구간을 사용하여 지연 시간을 줄이고 갱신 사이의 이동 거리를 제한할 수 있다. 적응형 정책(Adaptive Policy)은 모든 운용 조건에서 하나의 고정 프레임 지속 시간을 사용하는 대신 각속도, 병진 속도, 포인트 수, 추정기 계산 부하를 고려할 수 있다.

선택적 포인트 샘플링(Selective Point Sampling)을 통해 처리 부하를 제어할 수 있다. 추정기가 정확한 자세를 계산하기 위해 모든 LiDAR 반사점을 사용할 필요는 없다. 복셀 샘플링, 기하학적 중요도(Geometric Importance), 거리, 표면 방향, 공간적 균일성(Spatial Uniformity)을 기준으로 최적화에 참여할 측정값을 선택할 수 있다. 동일한 표면의 중복 관측이 대부분을 차지하는 대규모 포인트 집합보다 크기가 작더라도 기하학적으로 다양한 포인트 집합이 더 강한 제약을 제공할 수 있다.

지속적인 실시간 운용을 위해서는 로컬 지도 관리(Local-Map Management)가 필수적이다. 활성 지도(Active Map)는 스캔 대 지도 정합에 필요한 영역을 포함하면서 로봇에서 멀리 떨어진 기하 구조를 제거해야 한다. 증분 복셀 구조(Incremental Voxel Structure)를 사용하면 전체 포인트 클라우드를 다시 구성하지 않고도 포인트를 삽입하고 삭제할 수 있다. 장거리 운용에서는 완료된 영역을 서브맵(Submap), 타일 저장소(Tiled Storage) 또는 전역 매핑 계층(Global Mapping Layer)으로 전달하고 오도메트리 프론트엔드는 주변 기하 정보만 유지할 수 있다.

솔리드 스테이트 LiDAR 오도메트리(Solid-State LiDAR Odometry)는 개념적으로 전역 SLAM(Global SLAM)과 구분해야 한다. 고주파 LiDAR-관성 추정은 부드러운 로컬 움직임을 제공하지만 긴 궤적에서는 잔여 드리프트가 발생한다. 루프 폐쇄(Loop Closure), GNSS 또는 RTK, 측량된 랜드마크(Surveyed Landmark), 포즈 그래프 최적화(Pose-Graph Optimization)를 이용하여 전역 보정을 제공할 수 있다. 이러한 계층형 아키텍처는 로컬 추정기의 계산량을 제한하면서 상대적으로 느린 프로세스가 대규모 지도 일관성을 유지하도록 한다.

루프 검출(Loop Detection)은 개별적인 짧은 프레임 대신 누적된 MID-360 관측으로 생성된 키프레임 포인트 클라우드(Keyframe Point Cloud)를 사용할 수 있다. 키프레임은 장소 인식(Place Recognition)과 검증에 사용할 수 있는 보다 조밀하고 특징적인 로컬 기하 구조를 제공한다. 그러나 반복 구조는 여전히 잘못된 후보를 생성할 수 있으므로 루프 제약에는 기하학적 검증과 일관성 검사가 필요하다. 전역 최적화에서는 디스크립터 일치만으로 동일한 공간이라고 판단해서는 안 된다.

야외 운용에서는 비, 먼지, 안개, 반사 표면, 직사광선, 진동, 온도 변화와 같은 환경 요인이 존재한다. 이러한 요소의 영향은 센서 특성과 운용 조건에 따라 달라지므로 전처리 과정에서는 정상적인 기하 정보를 불필요하게 제거하지 않으면서 명확하게 신뢰성이 낮은 측정값을 제거해야 한다. 추정기 상태 감시(Estimator-Health Monitoring)를 통해 비정상적인 포인트 수, 잔차 증가, 시간 동기화 문제 또는 정합 품질의 급격한 변화를 검출할 수 있다.

따라서 실용적인 MID-360 SLAM 파이프라인은 동기화된 LiDAR 및 IMU 데이터 획득에서 시작하여 포인트 수준의 타임스탬프 처리, 모션 보상, 자체 포인트 필터링, 공간 샘플링으로 이어진다. 관성 전파는 상태를 예측하고 스캔 대 지도 기하 잔차가 이를 보정하며 수용된 관측값은 증분 복셀 지도(Incremental Voxel Map)를 갱신한다. 퇴화 검출과 추정기 상태 검사를 통해 최종 자세 추정값을 어느 정도 신뢰할 것인지 결정한다.

야외 자율이동로봇의 경우 이러한 로컬 파이프라인을 임무 요구사항에 따라 휠 오도메트리, GNSS/RTK, 카메라 또는 레이더와 연결할 수 있다. MID-360은 조밀한 3차원 환경 기하 정보를 제공하고 IMU는 고주파 운동 연속성을 유지하며 외부 센서는 상호 보완적인 제약을 제공한다. 센서 가중치는 모든 지형과 운용 조건에서 고정된 값으로 유지하기보다 실제 측정 품질과 관측 가능성을 반영해야 한다.

따라서 MID-360에 SLAM을 적용하는 것은 단순히 기존 LiDAR 드라이버를 다른 드라이버로 교체하는 문제가 아니다. 스캐닝 패턴의 변화는 프레임, 이웃 구조, 시간 처리, 특징 추출, 포인트 밀도, 지도 갱신에 관한 기존 가정을 변화시킨다. 강인한 구현은 이러한 특성을 추정기 설계의 일부로 취급하고 타임스탬프 인식 전처리(Timestamp-Aware Preprocessing), 긴밀 결합형 관성 추정, 복셀 매핑, 관측 가능성 감시, 전역 보정을 통합해야 한다.

이러한 적응이 올바르게 구현되면 소형 솔리드 스테이트 LiDAR는 이동 로봇을 위한 강력한 인지 및 위치 추정 기반을 제공할 수 있다. 결과적인 아키텍처는 기존 스캔 링 가정에 의존하지 않으면서 넓은 3차원 커버리지를 활용할 수 있으며 증분 매핑을 통해 계산량을 제한된 수준으로 유지할 수 있다. 신뢰성 높은 보정, 시간 동기화, 센서 융합, 전역 SLAM과 결합하면 MID-360 기반 위치 추정은 확장 가능한 실내·야외·산업용 자율 시스템을 지원할 수 있다.

##  

## 06.09. LiDAR SLAM Long Term Map Drift Correction [w/Code]

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

Long-term map drift is the gradual loss of global spatial consistency that occurs when small localization errors accumulate over extended LiDAR SLAM trajectories. Even highly accurate scan matching introduces residual translation and rotation errors. When thousands of incremental pose estimates are chained together, these small errors can become meter-scale displacement, heading error, duplicated structures, or visible deformation in a large three-dimensional map.

Local registration accuracy and global map consistency are therefore different objectives. Scan-to-scan or scan-to-map matching may provide excellent short-term motion estimates while the complete trajectory slowly diverges from its true global geometry. A robot can remain locally well localized relative to nearby surfaces while distant portions of the same map no longer align correctly. Long-term correction must introduce constraints that connect separated portions of the trajectory.

Drift originates from several interacting sources. LiDAR measurement noise, imperfect correspondence, limited geometric observability, IMU bias, wheel slip, calibration error, timestamp offset, vibration, and dynamic objects can all contribute. Environmental structure also matters. Long corridors, tunnels, open roads, repetitive warehouses, and feature-poor areas may weakly constrain particular motion directions, allowing errors to accumulate faster than in geometrically diverse environments.

Rotational drift is especially important because a small heading error produces increasingly large position error as traveled distance increases. A slight yaw bias accumulated over a long outdoor route can shift the reconstructed map substantially even if individual scan alignments appear accurate. Roll and pitch errors can similarly distort elevation and vertical structure. Drift correction must therefore estimate complete pose relationships rather than only correcting Cartesian position.

Keyframes provide a practical representation for long-term correction. Instead of optimizing every LiDAR frame, selected poses are stored as graph nodes together with representative point clouds or map features. Consecutive keyframes are connected by odometry constraints derived from LiDAR-inertial estimation. Additional relationships can later be inserted between nonconsecutive keyframes when the system discovers that they correspond to the same location or share another reliable spatial reference.

A pose graph converts long-term mapping into a global constraint optimization problem. Nodes represent robot poses and edges represent measured relative or absolute relationships between them. Sequential LiDAR-inertial odometry forms the backbone of the graph, while loop closures, GNSS observations, surveyed landmarks, or other references add constraints that limit accumulated drift. Optimization adjusts many poses simultaneously to find a trajectory that best satisfies all accepted measurements.

Loop closure is one of the most powerful mechanisms for correcting accumulated drift. When the robot revisits a previously mapped area, current observations can be compared with historical keyframes or submaps. A verified match creates a constraint between two trajectory locations that may be separated by minutes, hours, or kilometers of travel. The optimizer can then redistribute accumulated error across the intervening trajectory instead of applying an abrupt correction only at the current pose.

Loop candidate generation should be separated from geometric verification. Place-recognition descriptors, spatial proximity, GNSS estimates, intensity patterns, or learned representations can identify possible revisits efficiently. Candidate generation should favor recall, while a later verification stage determines whether the relationship is sufficiently reliable for optimization. This separation avoids performing expensive point-cloud registration against every historical keyframe.

Geometric verification can use ICP, generalized ICP, NDT, voxelized registration, or other point-cloud alignment techniques to estimate the relative transformation between candidate regions. A valid loop should demonstrate sufficient geometric overlap, stable convergence, reasonable transformation, and residual consistency. The registration result should also agree with broader trajectory constraints. A low local alignment error alone does not guarantee that two observations represent the same physical place.

False loop closures are particularly dangerous in long-term mapping. Repetitive warehouses, tunnels, parking structures, roads, and industrial facilities can contain spatially distinct regions with nearly identical geometry. An incorrect loop edge can cause the optimizer to deform a large section of an otherwise accurate map. Conservative validation, temporal consistency, multiple consecutive matches, semantic information, and robust graph optimization are therefore important for operational reliability.

Robust kernels reduce the influence of graph constraints whose residuals become unexpectedly large during optimization. Huber, Cauchy, or related loss functions prevent a questionable measurement from dominating the global solution. More advanced approaches can assign switchable weights or reject inconsistent loop edges. Robust optimization does not replace careful loop verification, but it provides an additional protection layer when uncertain constraints enter a large pose graph.

GNSS and RTK provide another important source of long-term drift correction for outdoor LiDAR SLAM. Unlike relative odometry, global navigation measurements directly relate the trajectory to an external geographic reference. High-quality RTK observations can prevent slow positional drift over long routes and help align independently generated mapping sessions. Their uncertainty must still be modeled because satellite visibility, multipath, obstruction, and correction-link quality vary with location.

Global measurements should be weighted according to actual quality rather than treated as perfectly accurate anchors. GNSS covariance, fix type, satellite geometry, correction age, and consistency with inertial motion can influence confidence. Measurements obtained near buildings, trees, tunnels, bridges, or reflective infrastructure may require reduced weighting or rejection. Adaptive confidence prevents unreliable global observations from damaging an otherwise consistent LiDAR trajectory.

IMU information contributes strongly to local continuity but also influences long-term map quality. Bias errors in angular velocity and acceleration can slowly alter orientation, velocity, and position estimates. Tightly coupled LiDAR-inertial estimation repeatedly corrects these errors using geometry, but weakly observable environments may allow some bias-related drift to persist. Global graph constraints indirectly improve the trajectory by correcting the accumulated consequences of these local estimation errors.

Calibration stability is essential because systematic sensor errors cannot be solved reliably through loop closure alone. Incorrect LiDAR-to-IMU extrinsics, timestamp offsets, or moving sensor mounts introduce repeatable trajectory errors. A global optimizer may distribute these errors throughout the graph without eliminating their physical cause. Long-term systems should therefore monitor calibration validity and distinguish systematic estimation problems from ordinary stochastic drift.

Submaps can improve scalability by grouping many local observations into bounded map segments. Each submap maintains internally consistent geometry, while the global optimizer estimates transformations between submaps. Loop closures can then connect submaps rather than individual scans. This reduces graph complexity and supports very large environments, multi-session mapping, and long-duration autonomous operation without requiring every historical measurement to remain active in memory.

Hierarchical optimization extends this concept further. A local estimator handles high-rate LiDAR-inertial motion, a regional layer manages keyframes or submaps, and a global layer resolves large-scale loop closures and geographic constraints. Each layer operates at a different frequency and spatial scale. Such separation allows real-time localization to continue smoothly while computationally heavier global corrections are processed asynchronously.

Applying a global correction requires careful coordination with the real-time control system. An optimized trajectory may move historical poses substantially, but abruptly changing the robot\'s control-frame pose can produce discontinuities that destabilize navigation. Many systems therefore maintain a smooth local odometry frame for control and a separate global map frame that can shift after optimization. A transform between these frames communicates global correction without breaking local motion continuity.

Map reconstruction follows pose optimization. If point clouds are permanently fused into a single immutable global cloud, correcting historical poses becomes difficult. A more flexible architecture associates keyframe or submap geometry with adjustable poses. After optimization, stored geometry can be transformed using corrected poses and regenerated into a globally consistent map. This design separates measurement storage from the current estimate of where those measurements belong.

Incremental optimization is useful because long-term robots continually receive new measurements. Re-solving the entire graph from scratch after every new keyframe or loop closure is computationally inefficient. Incremental graph methods reuse previous solutions and update only the affected portions of the estimation problem where possible. Periodic full optimization may still be useful for offline map refinement or when major global constraints are introduced.

Map drift correction must also account for environmental change. A location observed months later may contain new buildings, relocated equipment, vegetation growth, parked vehicles, or modified road geometry. Treating every difference as localization error can produce incorrect loop constraints. Long-term SLAM therefore benefits from separating persistent structural geometry from temporary or changed content and recording when map observations were acquired.

Multi-session mapping extends drift correction beyond a single robot run. New trajectories can be aligned with a persistent reference map using common landmarks, overlapping submaps, GNSS coordinates, or place recognition. Once reliable inter-session constraints are established, graph optimization can combine multiple sessions into a shared coordinate system. This allows the map to improve progressively rather than requiring complete remapping whenever the robot returns to the site.

Map versioning becomes important when corrected maps are deployed operationally. A global optimization may modify keyframe poses, tile boundaries, or localization geometry used by a robot fleet. The resulting map should therefore carry an explicit version and compatibility information. Robots should not unknowingly localize against inconsistent mixtures of old and newly optimized tiles. Controlled publication allows map correction to become a managed system update.

Correction quality should be evaluated using more than visual appearance. Loop residuals, trajectory consistency, GNSS agreement, absolute trajectory error, relative pose error, map overlap, structural alignment, and covariance can provide quantitative evidence. Independent surveyed landmarks are particularly valuable because they evaluate the map against an external reference rather than measurements already involved in the optimization process.

Estimator health monitoring can indicate when drift is becoming operationally significant before a loop closure is available. Growing covariance, weak geometric observability, disagreement between LiDAR and GNSS, repeated registration degradation, or unusual IMU bias estimates can trigger warnings. The robot may then reduce speed, seek a known localization region, increase reliance on external references, or schedule map maintenance before uncertainty exceeds mission limits.

For large outdoor AMRs, long-term correction should combine smooth high-rate local localization with slower global consistency management. LiDAR-inertial odometry can provide continuous six-degree-of-freedom motion, while RTK, loop closures, mapped landmarks, and pose-graph optimization constrain accumulated error. The system should preserve local control continuity while allowing the global map and mission coordinates to improve whenever stronger information becomes available.

Long-term drift correction is therefore not a single final optimization step performed after mapping. It is a continuous architecture for managing uncertainty across different temporal and spatial scales. Local estimation provides immediate motion, keyframes preserve important historical observations, loop detection discovers repeated places, external references anchor the trajectory, and graph optimization redistributes accumulated error while robust validation protects the solution from incorrect constraints.

A mature LiDAR SLAM system treats the map as an evolving spatial estimate rather than a fixed collection of points. Measurements, poses, uncertainty, timestamps, calibration information, and map versions remain connected so that new evidence can revise previous assumptions. This structure enables autonomous robots to maintain useful localization over long routes, repeated missions, and changing environments while preserving both real-time performance and global spatial consistency.

장기 지도 드리프트(Long-Term Map Drift)는 장시간의 LiDAR SLAM 궤적에서 작은 위치 추정 오차가 누적되면서 전역 공간 일관성(Global Spatial Consistency)이 점진적으로 손실되는 현상이다. 매우 정확한 스캔 정합(Scan Matching)에서도 미세한 병진 및 회전 오차가 남을 수 있다. 수천 개의 증분 자세 추정값(Incremental Pose Estimate)이 연속적으로 연결되면 이러한 작은 오차가 미터 단위 위치 편차, 방향 오차, 구조물 중복 또는 대규모 3차원 지도의 가시적인 변형으로 발전할 수 있다.

따라서 로컬 정합 정확도(Local Registration Accuracy)와 전역 지도 일관성(Global Map Consistency)은 서로 다른 목표이다. 스캔 대 스캔(Scan-to-Scan) 또는 스캔 대 지도(Scan-to-Map) 정합은 매우 우수한 단기 운동 추정값을 제공하면서도 전체 궤적은 실제 전역 기하 구조에서 서서히 벗어날 수 있다. 로봇은 주변 표면에 대해서는 정확하게 위치를 추정하면서도 동일한 지도의 멀리 떨어진 영역들이 서로 정확하게 정렬되지 않을 수 있다. 장기 보정(Long-Term Correction)은 서로 떨어진 궤적 영역을 연결하는 제약을 추가해야 한다.

드리프트는 여러 원인이 상호작용하여 발생한다. LiDAR 측정 노이즈, 불완전한 대응점(Correspondence), 제한된 기하학적 관측 가능성(Geometric Observability), IMU 바이어스, 휠 슬립(Wheel Slip), 보정 오차(Calibration Error), 타임스탬프 오프셋(Timestamp Offset), 진동, 동적 객체 등이 모두 영향을 줄 수 있다. 환경 구조 역시 중요하다. 긴 복도, 터널, 개방 도로, 반복적인 창고, 특징이 부족한 영역에서는 특정 운동 방향에 대한 제약이 약해져 기하학적으로 다양한 환경보다 오차가 빠르게 누적될 수 있다.

회전 드리프트(Rotational Drift)는 작은 방향 오차도 이동 거리가 증가하면서 큰 위치 오차로 확대되기 때문에 특히 중요하다. 긴 야외 경로에서 미세한 요 바이어스(Yaw Bias)가 누적되면 각각의 스캔 정합이 정확해 보이더라도 재구성된 지도가 크게 이동할 수 있다. 롤(Roll)과 피치(Pitch) 오차 역시 고도와 수직 구조를 왜곡할 수 있다. 따라서 드리프트 보정은 단순히 직교 좌표 위치만 수정하는 것이 아니라 전체 자세 관계(Full Pose Relationship)를 추정해야 한다.

키프레임(Keyframe)은 장기 보정을 위한 실용적인 표현을 제공한다. 모든 LiDAR 프레임을 최적화하는 대신 선택된 자세를 대표 포인트 클라우드 또는 지도 특징과 함께 그래프 노드(Graph Node)로 저장한다. 연속적인 키프레임은 LiDAR-관성 추정(LiDAR-Inertial Estimation)으로부터 얻어진 오도메트리 제약(Odometry Constraint)으로 연결된다. 이후 동일한 위치에 대응하거나 신뢰할 수 있는 다른 공간 기준을 공유한다는 사실이 발견되면 비연속 키프레임 사이에 추가적인 관계를 삽입할 수 있다.

포즈 그래프(Pose Graph)는 장기 매핑을 전역 제약 최적화(Global Constraint Optimization) 문제로 변환한다. 노드는 로봇 자세를 나타내고 에지(Edge)는 노드 사이에서 측정된 상대 또는 절대 관계를 나타낸다. 연속적인 LiDAR-관성 오도메트리는 그래프의 기본 골격을 형성하며 루프 폐쇄(Loop Closure), GNSS 관측, 측량된 랜드마크(Surveyed Landmark) 또는 기타 기준 정보가 누적 드리프트를 제한하는 추가 제약을 제공한다. 최적화는 수용된 모든 측정값을 가장 잘 만족하도록 여러 자세를 동시에 조정한다.

루프 폐쇄는 누적된 드리프트를 보정하기 위한 가장 강력한 방법 중 하나이다. 로봇이 이전에 지도화한 영역을 다시 방문하면 현재 관측값을 과거의 키프레임 또는 서브맵(Submap)과 비교할 수 있다. 검증된 일치 관계는 수분, 수시간 또는 수 킬로미터의 이동으로 분리된 두 궤적 위치 사이에 제약을 생성한다. 이후 최적화기는 현재 자세에만 갑작스럽게 보정을 적용하는 대신 그 사이의 전체 궤적에 누적된 오차를 재분배할 수 있다.

루프 후보 생성(Loop Candidate Generation)은 기하학적 검증(Geometric Verification)과 분리하는 것이 바람직하다. 장소 인식 디스크립터(Place-Recognition Descriptor), 공간적 근접성, GNSS 추정값, 반사 강도 패턴(Intensity Pattern), 학습 기반 표현(Learned Representation)을 이용하여 가능한 재방문 위치를 효율적으로 식별할 수 있다. 후보 생성 단계에서는 재현율(Recall)을 높이고 이후 검증 단계에서 해당 관계가 최적화에 사용할 만큼 충분히 신뢰할 수 있는지를 판단한다. 이러한 분리는 모든 과거 키프레임에 대해 계산 비용이 높은 포인트 클라우드 정합을 수행하는 것을 방지한다.

기하학적 검증에는 ICP, 일반화 ICP(Generalized ICP), NDT, 복셀 기반 정합(Voxelized Registration) 또는 다른 포인트 클라우드 정렬 기법을 사용하여 후보 영역 사이의 상대 변환을 추정할 수 있다. 유효한 루프는 충분한 기하 중첩(Geometric Overlap), 안정적인 수렴, 합리적인 변환, 잔차 일관성을 보여야 한다. 또한 정합 결과는 전체 궤적의 다른 제약과도 일치해야 한다. 낮은 로컬 정합 오차만으로 두 관측이 동일한 물리적 위치를 나타낸다고 보장할 수는 없다.

잘못된 루프 폐쇄(False Loop Closure)는 장기 매핑에서 특히 위험하다. 반복적인 창고, 터널, 주차 구조물, 도로, 산업 시설에서는 공간적으로 서로 다른 영역이 거의 동일한 기하 구조를 가질 수 있다. 하나의 잘못된 루프 에지는 원래 정확했던 지도의 넓은 영역을 최적화 과정에서 변형시킬 수 있다. 따라서 보수적인 검증, 시간적 일관성(Temporal Consistency), 연속된 여러 일치 결과, 의미론적 정보(Semantic Information), 강인 그래프 최적화(Robust Graph Optimization)가 실제 운용 신뢰성을 위해 중요하다.

강인 커널(Robust Kernel)은 최적화 과정에서 예상보다 큰 잔차를 갖는 그래프 제약의 영향을 감소시킨다. 후버(Huber), 코시(Cauchy) 또는 이와 유사한 손실 함수는 의심스러운 측정값 하나가 전역 해(Global Solution)를 지배하지 못하도록 한다. 보다 발전된 방식에서는 전환 가능한 가중치(Switchable Weight)를 적용하거나 일관되지 않는 루프 에지를 제거할 수 있다. 강인 최적화는 세심한 루프 검증을 대체하지 않지만 불확실한 제약이 대규모 포즈 그래프에 입력되었을 때 추가적인 보호 계층을 제공한다.

GNSS와 RTK는 야외 LiDAR SLAM의 장기 드리프트 보정을 위한 또 다른 중요한 정보원이다. 상대 오도메트리와 달리 전역 항법 측정(Global Navigation Measurement)은 궤적을 외부의 지리적 기준과 직접 연결한다. 고품질 RTK 관측은 장거리 경로에서 느리게 누적되는 위치 드리프트를 억제하고 서로 독립적으로 생성된 여러 매핑 세션(Mapping Session)을 정렬하는 데 도움을 줄 수 있다. 그러나 위성 가시성, 다중경로(Multipath), 차폐, 보정 링크 품질이 위치에 따라 달라지므로 불확실성을 적절하게 모델링해야 한다.

전역 측정값(Global Measurement)은 완벽하게 정확한 앵커(Anchor)로 간주하기보다 실제 품질에 따라 가중치를 부여해야 한다. GNSS 공분산, 고정해 유형(Fix Type), 위성 기하 구조(Satellite Geometry), 보정 정보의 경과 시간(Correction Age), 관성 운동과의 일관성 등을 이용하여 신뢰도를 결정할 수 있다. 건물, 나무, 터널, 교량 또는 반사 구조 주변에서 획득한 측정값은 가중치를 낮추거나 제거해야 할 수 있다. 적응형 신뢰도(Adaptive Confidence)는 신뢰성이 낮은 전역 관측값이 이미 일관된 LiDAR 궤적을 손상시키는 것을 방지한다.

IMU 정보는 로컬 연속성(Local Continuity)에 크게 기여하지만 장기 지도 품질에도 영향을 준다. 각속도와 가속도의 바이어스 오차는 자세, 속도, 위치 추정값을 점진적으로 변화시킬 수 있다. 긴밀 결합형 LiDAR-관성 추정(Tightly Coupled LiDAR-Inertial Estimation)은 기하 정보를 이용하여 이러한 오차를 반복적으로 보정하지만 관측 가능성이 낮은 환경에서는 일부 바이어스 관련 드리프트가 지속될 수 있다. 전역 그래프 제약은 이러한 로컬 추정 오차가 장기적으로 누적된 결과를 보정함으로써 전체 궤적을 개선한다.

체계적인 센서 오차(Systematic Sensor Error)는 루프 폐쇄만으로 안정적으로 해결하기 어렵기 때문에 보정 안정성(Calibration Stability)이 필수적이다. 잘못된 LiDAR-IMU 외부 파라미터(Extrinsic Parameter), 타임스탬프 오프셋 또는 움직이는 센서 마운트는 반복적인 궤적 오차를 발생시킨다. 전역 최적화기는 이러한 오차를 그래프 전체에 분산시킬 수 있지만 물리적인 원인 자체를 제거하지는 못한다. 따라서 장기 운용 시스템은 보정 유효성을 감시하고 체계적인 추정 문제와 일반적인 확률적 드리프트(Stochastic Drift)를 구분해야 한다.

서브맵(Submap)은 여러 로컬 관측을 제한된 지도 세그먼트(Map Segment)로 묶어 확장성을 향상시킬 수 있다. 각각의 서브맵은 내부적으로 일관된 기하 구조를 유지하며 전역 최적화기는 서브맵 사이의 변환을 추정한다. 이후 루프 폐쇄는 개별 스캔이 아니라 서브맵 사이를 연결할 수 있다. 이를 통해 그래프 복잡도를 감소시키고 모든 과거 측정값을 메모리에서 활성 상태로 유지하지 않고도 대규모 환경, 다중 세션 매핑(Multi-Session Mapping), 장시간 자율 운용을 지원할 수 있다.

계층형 최적화(Hierarchical Optimization)는 이러한 개념을 더욱 확장한다. 로컬 추정기는 고주파 LiDAR-관성 운동을 처리하고 지역 계층(Regional Layer)은 키프레임 또는 서브맵을 관리하며 전역 계층(Global Layer)은 대규모 루프 폐쇄와 지리적 제약을 처리한다. 각각의 계층은 서로 다른 주기와 공간 규모에서 동작한다. 이러한 분리를 통해 계산 비용이 높은 전역 보정이 비동기적으로 처리되는 동안에도 실시간 위치 추정을 부드럽게 지속할 수 있다.

전역 보정(Global Correction)을 적용할 때에는 실시간 제어 시스템과의 조정이 필요하다. 최적화된 궤적은 과거 자세를 크게 변경할 수 있지만 로봇의 제어 좌표계(Control Frame) 자세를 갑자기 변경하면 불연속성이 발생하여 내비게이션을 불안정하게 만들 수 있다. 따라서 많은 시스템에서는 제어를 위한 부드러운 로컬 오도메트리 좌표계(Local Odometry Frame)와 최적화 이후 이동할 수 있는 별도의 전역 지도 좌표계(Global Map Frame)를 유지한다. 두 좌표계 사이의 변환을 통해 로컬 운동 연속성을 손상시키지 않으면서 전역 보정 정보를 전달한다.

지도 재구성(Map Reconstruction)은 자세 최적화 이후 수행된다. 포인트 클라우드를 하나의 변경 불가능한 전역 클라우드에 영구적으로 융합하면 과거 자세를 수정하기 어렵다. 보다 유연한 아키텍처에서는 키프레임 또는 서브맵 기하 구조를 조정 가능한 자세와 연결하여 저장한다. 최적화 이후 저장된 기하 정보를 보정된 자세를 이용하여 다시 변환하고 전역적으로 일관된 지도를 재생성할 수 있다. 이러한 설계는 측정 데이터의 저장과 해당 측정값이 어느 위치에 속하는지에 대한 현재 추정값을 분리한다.

장기간 운용하는 로봇은 지속적으로 새로운 측정값을 입력받기 때문에 증분 최적화(Incremental Optimization)가 유용하다. 새로운 키프레임이나 루프 폐쇄가 추가될 때마다 전체 그래프를 처음부터 다시 계산하는 것은 계산적으로 비효율적이다. 증분 그래프 방식은 이전 해를 재사용하고 가능한 경우 추정 문제에서 영향을 받는 영역만 갱신한다. 그러나 오프라인 지도 정제(Offline Map Refinement) 또는 중요한 전역 제약이 추가된 경우에는 주기적인 전체 최적화(Full Optimization)가 여전히 유용할 수 있다.

지도 드리프트 보정은 환경 변화(Environmental Change)도 고려해야 한다. 수개월 후 동일한 장소를 다시 관측하면 새로운 건물, 이동된 장비, 성장한 식생, 주차 차량 또는 변경된 도로 구조가 존재할 수 있다. 모든 차이를 위치 추정 오차로 처리하면 잘못된 루프 제약이 생성될 수 있다. 따라서 장기 SLAM에서는 지속적인 구조 기하 정보(Persistent Structural Geometry)를 일시적이거나 변경된 객체와 분리하고 지도 관측이 획득된 시점을 기록하는 것이 유용하다.

다중 세션 매핑(Multi-Session Mapping)은 드리프트 보정을 하나의 로봇 주행 이상으로 확장한다. 새로운 궤적은 공통 랜드마크, 중첩 서브맵, GNSS 좌표 또는 장소 인식을 이용하여 지속적으로 유지되는 기준 지도(Persistent Reference Map)에 정렬할 수 있다. 신뢰할 수 있는 세션 간 제약(Inter-Session Constraint)이 확보되면 그래프 최적화를 통해 여러 세션을 하나의 공유 좌표계(Shared Coordinate System)로 통합할 수 있다. 이를 통해 로봇이 현장에 다시 방문할 때마다 전체 지도를 처음부터 다시 생성하지 않고 점진적으로 개선할 수 있다.

보정된 지도를 실제 운용 시스템에 배포할 때에는 지도 버전 관리(Map Versioning)가 중요하다. 전역 최적화는 키프레임 자세, 타일 경계(Tile Boundary), 로봇 플릿(Robot Fleet)이 사용하는 위치 추정용 기하 정보를 변경할 수 있다. 따라서 생성된 지도에는 명시적인 버전과 호환성 정보(Compatibility Information)가 포함되어야 한다. 로봇이 기존 타일과 새롭게 최적화된 타일이 혼합된 불일치 지도를 자신도 모르게 사용해서는 안 된다. 제어된 배포(Controlled Publication)를 통해 지도 보정을 관리 가능한 시스템 업데이트로 처리할 수 있다.

보정 품질(Correction Quality)은 지도의 시각적 외형만으로 평가해서는 안 된다. 루프 잔차(Loop Residual), 궤적 일관성, GNSS와의 일치도, 절대 궤적 오차(Absolute Trajectory Error), 상대 자세 오차(Relative Pose Error), 지도 중첩도(Map Overlap), 구조 정렬(Structural Alignment), 공분산 등을 통해 정량적으로 평가할 수 있다. 독립적으로 측량된 랜드마크는 최적화에 이미 사용된 측정값이 아닌 외부 기준을 이용하여 지도를 평가할 수 있기 때문에 특히 가치가 높다.

추정기 상태 감시(Estimator Health Monitoring)는 루프 폐쇄가 아직 발생하지 않았더라도 드리프트가 운용상 중요한 수준으로 증가하고 있음을 감지할 수 있다. 증가하는 공분산, 약한 기하학적 관측 가능성, LiDAR와 GNSS 사이의 불일치, 반복적인 정합 성능 저하, 비정상적인 IMU 바이어스 추정값 등이 경고 조건이 될 수 있다. 로봇은 불확실성이 임무 허용 범위를 초과하기 전에 속도를 낮추거나 알려진 위치 추정 영역으로 이동하거나 외부 기준에 대한 의존도를 높이거나 지도 유지보수를 수행할 수 있다.

대형 야외 자율이동로봇(Outdoor AMR)의 장기 보정은 부드러운 고주파 로컬 위치 추정과 상대적으로 느린 전역 일관성 관리를 결합해야 한다. LiDAR-관성 오도메트리는 연속적인 6자유도 운동을 제공하고 RTK, 루프 폐쇄, 지도화된 랜드마크, 포즈 그래프 최적화는 누적된 오차를 제한한다. 시스템은 로컬 제어의 연속성을 유지하면서 더 강한 정보가 확보될 때마다 전역 지도와 임무 좌표(Mission Coordinate)를 개선할 수 있어야 한다.

따라서 장기 드리프트 보정(Long-Term Drift Correction)은 매핑이 완료된 이후 한 번 수행하는 최종 최적화 단계가 아니다. 이는 서로 다른 시간적·공간적 규모에서 불확실성을 지속적으로 관리하기 위한 아키텍처이다. 로컬 추정(Local Estimation)은 즉각적인 운동 정보를 제공하고 키프레임은 중요한 과거 관측을 보존하며 루프 검출은 재방문 장소를 발견한다. 외부 기준은 궤적을 고정하고 그래프 최적화는 누적 오차를 재분배하며 강인한 검증은 잘못된 제약으로부터 전체 해를 보호한다.

성숙한 LiDAR SLAM 시스템은 지도를 고정된 포인트의 집합이 아니라 지속적으로 변화하고 개선되는 공간 추정값(Evolving Spatial Estimate)으로 취급한다. 측정값, 자세, 불확실성, 타임스탬프, 보정 정보, 지도 버전이 서로 연결된 상태로 유지되어 새로운 정보가 과거의 가정을 수정할 수 있어야 한다. 이러한 구조를 통해 자율 로봇은 실시간 성능과 전역 공간 일관성을 모두 유지하면서 장거리 경로, 반복 임무, 변화하는 환경에서도 유용하고 신뢰성 높은 위치 추정을 지속할 수 있다.

##  

## 06.10. LiDAR SLAM AMR Fleet Map Sharing Case

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

AMR fleet map sharing extends LiDAR SLAM from a single-robot localization problem into a distributed spatial-information system. Instead of every robot independently building and maintaining its own map, multiple AMRs operate against a shared reference while contributing observations from different routes and times. The objective is to preserve consistent localization across the fleet while controlling map updates, bandwidth, versions, and operational risk.

A practical fleet architecture separates local real-time localization from shared map management. Each AMR continuously performs LiDAR-inertial odometry and scan-to-map localization using map data stored locally on the vehicle. A central or on-premise map service maintains the authoritative global representation. This separation allows robots to continue navigating during temporary network interruptions without requiring every LiDAR frame to be transmitted to a server.

The shared map should therefore be treated as an operational product rather than a raw point-cloud repository. It can contain localization geometry, occupancy information, traversability layers, semantic landmarks, restricted areas, and metadata describing coordinate frames and quality. Dense archival point clouds may remain offline, while robots receive optimized representations containing only the information required for localization, navigation, and mission execution.

Large environments benefit from tiled or submap-based storage. A factory, logistics center, campus, port, hospital complex, or industrial site can be divided into spatial regions that are loaded independently. Each AMR downloads only the tiles around its current position and planned route. This reduces onboard memory requirements and network traffic while allowing the complete fleet map to become much larger than any individual robot\'s active localization map.

Every robot must share a common global coordinate convention. Local LiDAR-inertial odometry can maintain a smooth vehicle-centered trajectory, while a global map frame defines fleet-level positions, routes, landmarks, and mission coordinates. A transform between local odometry and global map frames allows global corrections without abruptly changing the pose used by the motion controller. This distinction becomes essential when maps are optimized or updated during operation.

Initial localization determines where a robot belongs within the shared map. GNSS or RTK can provide coarse initialization outdoors, while indoor systems may use known deployment stations, fiducial markers, place recognition, or global point-cloud registration. Once a reliable initial hypothesis is established, local scan-to-map matching refines the pose. Uncertain initialization should be explicitly detected because selecting the wrong map region can produce apparently stable but incorrect localization.

Fleet robots naturally provide repeated observations of the same environment. One AMR may traverse a loading area in the morning, another may pass through it later, and several robots may repeatedly observe major corridors or outdoor roads. These repeated measurements create an opportunity to identify stable geometry, detect environmental change, improve coverage, and estimate localization quality more reliably than a single mapping session.

However, observations from operational robots should not automatically modify the authoritative map. A temporary pallet, parked truck, pedestrian, construction barrier, opened door, or vegetation movement should not immediately become permanent localization geometry. Robot observations should first enter a staging or candidate-update layer where persistence, confidence, spatial consistency, and agreement across multiple observations can be evaluated.

Multi-session consistency is therefore a core requirement. New observations must be aligned with the current global map before they are considered for integration. LiDAR registration, GNSS references, known landmarks, and trajectory constraints can estimate this alignment. If a new session disagrees strongly with trusted map regions, the system should determine whether the cause is localization drift, calibration error, environmental change, or an incorrect global association before modifying shared data.

Map-change detection can compare current observations with persistent reference geometry. Repeated disappearance of an old structure or repeated appearance of a new structure across several robots may indicate a genuine environmental modification. Confidence should increase when independent robots observe the same change at different times. This fleet-level evidence helps distinguish permanent infrastructure changes from temporary objects and isolated sensor errors.

The authoritative map should use explicit version control. Each released map or tile can carry a version identifier, creation time, coordinate-frame definition, calibration compatibility, and quality status. Robots report which version they are currently using. Fleet management can then prevent missions from unknowingly mixing incompatible map products and can determine which vehicles require updates before entering newly modified regions.

Tile-level versioning reduces update cost. If only one loading dock or production area changes, the entire facility map does not need to be redistributed. Updated tiles can be validated and published independently while unaffected regions retain their existing versions. Dependency metadata can identify neighboring tiles that must be updated together when geometry crosses boundaries or when a global optimization changes their relative alignment.

Map publication should follow a controlled lifecycle. Candidate geometry is collected, aligned, filtered, and evaluated before becoming a validated map revision. Automated checks can test overlap, point density, structural continuity, localization performance, and coordinate consistency. Safety-critical deployments may additionally require human approval or offline replay before release. Only after validation should the new version become available to operational robots.

Rolling deployment can reduce fleet risk. Instead of updating every AMR simultaneously, a new map version can first be assigned to a small validation group. Localization residuals, mission completion, pose stability, and recovery events can then be compared with robots using the previous map. If performance remains acceptable, deployment expands gradually. A problematic version can be stopped before affecting the entire fleet.

Rollback capability is equally important. Robots should retain or be able to retrieve a previously validated map version when a new release causes unexpected localization problems. The map service should preserve version history and compatibility information so that rollback is deterministic rather than an emergency manual reconstruction. Map management therefore resembles software release management as much as conventional SLAM processing.

Network communication should prioritize compact map products rather than continuous raw LiDAR streaming. Tile identifiers, compressed voxel maps, keyframes, descriptors, change summaries, and selected diagnostic observations can be exchanged efficiently. Full-resolution point clouds may be uploaded only when needed for map reconstruction or investigation. This reduces bandwidth requirements and makes large fleets more practical over Wi-Fi, private 5G, or mixed networks.

Prefetching improves operation when robots move between map regions. The fleet or navigation system already knows much of the planned route, so required tiles can be downloaded before the AMR reaches them. Vehicle speed, route uncertainty, storage latency, and network quality determine how far ahead data should be cached. If the route changes unexpectedly, neighboring tiles provide a secondary buffer until the new path data becomes available.

Local caching is necessary for network resilience. Each robot should retain enough map information to continue operating through short communication outages. Frequently used tiles, the current mission corridor, recovery areas, and nearby alternatives can remain onboard. The system should distinguish between loss of network connectivity and loss of localization itself; a robot with a valid cached map may remain fully localized even when the central server is temporarily unreachable.

Map-sharing architecture should also support localization-health reporting. Robots can send compact statistics such as scan-matching residuals, map overlap, covariance, degeneracy indicators, GNSS disagreement, and relocalization frequency. Aggregating these metrics spatially reveals regions where many robots experience weak localization. Fleet-wide evidence can identify problematic map areas that may not be obvious from any single robot trajectory.

Such localization-quality maps can influence mission planning. If a particular corridor consistently produces poor LiDAR observability, routes can favor better-constrained areas when operationally reasonable. Robots entering weak regions may reduce speed, increase reliance on wheel odometry or inertial estimation, or search for mapped landmarks. The shared map can therefore represent not only geometry but also expected localization confidence.

Heterogeneous fleets introduce additional challenges because robots may use different LiDARs, mounting heights, fields of view, IMUs, or vehicle dimensions. The common map should contain geometry observable from multiple platforms rather than features visible only to one sensor configuration. Sensor-specific localization layers may still be generated when necessary, while all platforms remain aligned to the same global coordinate system.

Calibration metadata becomes important in such fleets. A map generated with one sensor configuration may contain systematic characteristics that influence another platform differently. Robots should report sensor models, extrinsic calibration versions, and relevant preprocessing configurations when contributing mapping observations. This information supports traceability when a suspicious map update or localization degradation appears after hardware maintenance.

Multi-robot loop closure can connect trajectories generated by different AMRs. If two robots observe the same distinctive location, verified inter-robot constraints can align their trajectories and improve global map consistency. These constraints require conservative validation because an incorrect cross-robot association can deform multiple sessions simultaneously. Geometric verification, timestamps, route context, and global references should support the decision.

Global pose-graph optimization can combine sequential odometry, intra-robot loop closures, inter-robot matches, GNSS or RTK measurements, and surveyed landmarks. The optimized result establishes a common spatial solution across the fleet. Because optimization may move historical submaps, operational systems should publish the result as a new controlled map version rather than silently altering coordinates while robots are executing missions.

A fleet map server can be organized into several logical services. A persistent repository stores validated maps and historical versions, a spatial index resolves positions and routes to tile identifiers, a distribution service delivers map data, and an update pipeline processes candidate observations. Monitoring services track localization quality and deployment state. These functions may run centrally or on-premise depending on latency, security, and site requirements.

Security and integrity are essential because shared maps directly influence physical robot motion. Robots should authenticate map sources, verify downloaded data, and reject corrupted or unauthorized versions. Access control should separate ordinary localization clients from systems allowed to propose, approve, or publish map updates. Audit records can preserve which robot contributed data, which process validated it, and when a version entered operational service.

Failure modes should be explicitly designed. If a required tile cannot be downloaded, the robot may use cached geometry, continue temporarily with LiDAR-inertial odometry, request an alternate route, reduce speed, or stop at a safe location. If a map version mismatch is detected, the system should avoid silently combining incompatible tiles. Predictable degraded behavior is preferable to maintaining mission speed with uncertain localization.

Fleet map sharing also enables progressive map improvement. Areas rarely visited during initial mapping can gain coverage as robots perform normal missions. Stable observations from multiple viewpoints can improve geometric completeness, while repeated trajectories reveal which structures are reliable localization references. The operational fleet effectively becomes a distributed sensing network, provided that map updates remain filtered and controlled.

For outdoor AMRs, GNSS/RTK can connect shared LiDAR maps to geographic coordinates and simplify alignment between distant regions. Local LiDAR-inertial localization remains important near buildings, under vegetation, inside tunnels, and in other GNSS-degraded areas. A shared architecture can therefore combine geographic anchoring with detailed local geometry while preserving one fleet-wide mission coordinate system.

The central engineering principle is that map sharing is not equivalent to allowing every robot to edit one common point cloud. Reliable fleet operation requires separation between local estimation, candidate observations, validated map products, version-controlled deployment, and rollback. This prevents temporary environmental changes or individual localization failures from immediately propagating to every vehicle in the system.

A mature AMR fleet therefore treats the shared LiDAR map as managed infrastructure. Robots consume spatial information, contribute evidence, report localization quality, and operate from locally cached validated versions. Central services integrate observations, detect change, optimize global consistency, and publish controlled revisions. This architecture allows LiDAR SLAM to scale from autonomous operation by one robot to coordinated localization across hundreds of mobile platforms.

AMR 플릿 지도 공유(AMR Fleet Map Sharing)는 LiDAR SLAM을 단일 로봇의 위치 추정 문제에서 분산형 공간 정보 시스템(Distributed Spatial-Information System)으로 확장한다. 각각의 로봇이 독립적으로 지도를 생성하고 유지하는 대신 여러 대의 AMR이 하나의 공유 기준 지도(Shared Reference Map)를 기반으로 운용되면서 서로 다른 경로와 시간에서 획득한 관측 정보를 제공한다. 핵심 목표는 지도 갱신, 네트워크 대역폭, 버전, 운용 위험을 관리하면서 전체 플릿에서 일관된 위치 추정을 유지하는 것이다.

실용적인 플릿 아키텍처(Fleet Architecture)는 로컬 실시간 위치 추정(Local Real-Time Localization)과 공유 지도 관리(Shared Map Management)를 분리한다. 각각의 AMR은 차량 내부에 저장된 지도 데이터를 사용하여 LiDAR-관성 오도메트리(LiDAR-Inertial Odometry)와 스캔 대 지도 위치 추정(Scan-to-Map Localization)을 지속적으로 수행한다. 중앙 또는 온프레미스 지도 서비스(On-Premise Map Service)는 권위 있는 전역 지도(Authoritative Global Map)를 관리한다. 이러한 분리를 통해 모든 LiDAR 프레임을 서버로 전송하지 않더라도 일시적인 네트워크 장애 중에 로봇이 계속 주행할 수 있다.

따라서 공유 지도(Shared Map)는 단순한 원시 포인트 클라우드 저장소(Raw Point-Cloud Repository)가 아니라 실제 운용을 위한 제품(Operational Product)으로 관리해야 한다. 지도에는 위치 추정용 기하 정보, 점유 정보(Occupancy Information), 주행 가능성 계층(Traversability Layer), 의미론적 랜드마크(Semantic Landmark), 제한 구역, 좌표계와 품질을 설명하는 메타데이터가 포함될 수 있다. 고밀도 보관용 포인트 클라우드는 오프라인에 유지하고 로봇에는 위치 추정, 내비게이션, 임무 수행에 필요한 정보만 포함하는 최적화된 표현을 제공할 수 있다.

대규모 환경에서는 타일 또는 서브맵 기반 저장(Tile- or Submap-Based Storage)이 유리하다. 공장, 물류센터, 캠퍼스, 항만, 병원 단지 또는 산업 현장을 독립적으로 로딩할 수 있는 공간 영역으로 분할할 수 있다. 각 AMR은 현재 위치와 계획된 경로 주변의 타일만 다운로드한다. 이를 통해 차량 내부 메모리 요구량과 네트워크 트래픽을 줄이면서 전체 플릿 지도는 개별 로봇의 활성 위치 추정 지도(Active Localization Map)보다 훨씬 큰 규모로 확장될 수 있다.

모든 로봇은 공통된 전역 좌표 규약(Global Coordinate Convention)을 공유해야 한다. 로컬 LiDAR-관성 오도메트리는 부드러운 차량 중심 궤적을 유지하고 전역 지도 좌표계(Global Map Frame)는 플릿 수준의 위치, 경로, 랜드마크, 임무 좌표를 정의한다. 로컬 오도메트리 좌표계와 전역 지도 좌표계 사이의 변환을 사용하면 모션 제어기(Motion Controller)가 사용하는 자세를 갑자기 변경하지 않으면서 전역 보정을 적용할 수 있다. 이러한 구분은 운용 중 지도가 최적화되거나 갱신될 때 특히 중요하다.

초기 위치 추정(Initial Localization)은 로봇이 공유 지도 내 어느 위치에 존재하는지를 결정한다. 야외에서는 GNSS 또는 RTK를 이용하여 대략적인 초기 위치를 제공할 수 있으며 실내에서는 알려진 배치 스테이션(Known Deployment Station), 기준 마커(Fiducial Marker), 장소 인식(Place Recognition), 전역 포인트 클라우드 정합(Global Point-Cloud Registration)을 사용할 수 있다. 신뢰할 수 있는 초기 가설이 확보되면 로컬 스캔 대 지도 정합을 통해 자세를 정밀화한다. 잘못된 지도 영역을 선택하면 안정적으로 보이지만 실제로는 잘못된 위치 추정이 발생할 수 있으므로 불확실한 초기화 상태를 명시적으로 검출해야 한다.

플릿 로봇은 자연스럽게 동일한 환경을 반복적으로 관측한다. 한 대의 AMR이 아침에 적재 구역을 통과하고 다른 로봇이 이후 동일한 구역을 지나갈 수 있으며 여러 로봇이 주요 복도나 야외 도로를 반복적으로 관측할 수 있다. 이러한 반복 측정(Repeated Observation)은 안정적인 기하 구조를 식별하고 환경 변화를 검출하며 지도 커버리지를 개선하고 단일 매핑 세션보다 위치 추정 품질을 더욱 신뢰성 있게 평가할 기회를 제공한다.

그러나 실제 운용 중인 로봇의 관측값이 권위 있는 지도(Authoritative Map)를 자동으로 수정하도록 해서는 안 된다. 일시적으로 놓인 팔레트, 주차된 트럭, 보행자, 공사 차단물, 열린 문 또는 움직이는 식생이 즉시 영구적인 위치 추정 기하 정보가 되어서는 안 된다. 로봇 관측값은 먼저 스테이징 또는 후보 갱신 계층(Staging or Candidate-Update Layer)에 입력되어 지속성(Persistence), 신뢰도, 공간적 일관성, 여러 관측 간 일치도를 평가해야 한다.

따라서 다중 세션 일관성(Multi-Session Consistency)은 핵심 요구사항이다. 새로운 관측값은 지도 통합 대상으로 고려되기 전에 현재 전역 지도와 정렬되어야 한다. LiDAR 정합, GNSS 기준, 알려진 랜드마크, 궤적 제약(Trajectory Constraint)을 이용하여 이러한 정렬을 추정할 수 있다. 새로운 세션이 신뢰할 수 있는 지도 영역과 크게 불일치하면 공유 데이터를 수정하기 전에 그 원인이 위치 추정 드리프트, 보정 오류, 환경 변화 또는 잘못된 전역 연관(Global Association)인지 판단해야 한다.

지도 변화 검출(Map-Change Detection)은 현재 관측값과 지속적인 기준 기하 구조(Persistent Reference Geometry)를 비교하여 수행할 수 있다. 기존 구조물이 반복적으로 사라지거나 새로운 구조물이 여러 로봇에 의해 반복적으로 관측되면 실제 환경 변화일 가능성이 높다. 서로 독립적인 로봇들이 서로 다른 시간에 동일한 변화를 관측하면 신뢰도를 더욱 높일 수 있다. 이러한 플릿 수준의 증거(Fleet-Level Evidence)는 영구적인 인프라 변화와 일시적 객체 또는 개별 센서 오류를 구분하는 데 도움을 준다.

권위 있는 지도는 명시적인 버전 관리(Version Control)를 사용해야 한다. 배포된 각각의 지도 또는 타일은 버전 식별자(Version Identifier), 생성 시간, 좌표계 정의, 보정 호환성(Calibration Compatibility), 품질 상태를 포함할 수 있다. 로봇은 현재 사용 중인 버전을 보고한다. 플릿 관리 시스템은 이를 통해 서로 호환되지 않는 지도 제품이 임무 중에 혼합되는 것을 방지하고 새롭게 수정된 영역에 진입하기 전에 어떤 차량이 업데이트되어야 하는지를 판단할 수 있다.

타일 수준 버전 관리(Tile-Level Versioning)는 업데이트 비용을 줄인다. 하나의 적재 도크나 생산 구역만 변경된 경우 시설 전체 지도를 다시 배포할 필요가 없다. 수정된 타일만 독립적으로 검증하고 배포하며 영향을 받지 않은 영역은 기존 버전을 유지할 수 있다. 종속성 메타데이터(Dependency Metadata)를 이용하면 기하 구조가 타일 경계를 통과하거나 전역 최적화로 상대 정렬이 변경될 때 함께 업데이트되어야 하는 인접 타일을 식별할 수 있다.

지도 배포(Map Publication)는 제어된 수명주기(Controlled Lifecycle)를 따라야 한다. 후보 기하 정보는 검증된 지도 개정판(Map Revision)이 되기 전에 수집, 정렬, 필터링, 평가 과정을 거친다. 자동화된 검사를 통해 중첩도, 포인트 밀도, 구조적 연속성, 위치 추정 성능, 좌표 일관성을 평가할 수 있다. 안전 중요도가 높은 시스템에서는 배포 전에 사람의 승인(Human Approval)이나 오프라인 재생 검증(Offline Replay)을 추가할 수 있다. 검증이 완료된 이후에만 새로운 버전을 실제 운용 로봇에 제공해야 한다.

점진적 배포(Rolling Deployment)는 플릿 전체의 위험을 줄일 수 있다. 모든 AMR을 동시에 새로운 지도 버전으로 업데이트하는 대신 먼저 소수의 검증 그룹(Validation Group)에 새로운 버전을 적용할 수 있다. 이후 위치 추정 잔차, 임무 완료 상태, 자세 안정성, 복구 이벤트 등을 이전 지도 버전을 사용하는 로봇과 비교할 수 있다. 성능이 허용 가능한 수준으로 유지되면 배포 범위를 점진적으로 확대한다. 문제가 있는 버전은 전체 플릿에 영향을 미치기 전에 배포를 중단할 수 있다.

롤백 기능(Rollback Capability) 역시 중요하다. 새로운 지도 버전이 예상하지 못한 위치 추정 문제를 발생시키는 경우 로봇은 이전에 검증된 지도 버전을 유지하거나 다시 불러올 수 있어야 한다. 지도 서비스는 버전 이력과 호환성 정보를 보존하여 롤백이 긴급 수동 재구성이 아니라 결정론적 과정(Deterministic Process)이 되도록 해야 한다. 따라서 지도 관리는 전통적인 SLAM 처리뿐 아니라 소프트웨어 릴리스 관리(Software Release Management)와 유사한 특성을 갖는다.

네트워크 통신(Network Communication)은 지속적인 원시 LiDAR 스트리밍보다 압축된 지도 데이터(Compact Map Product)를 우선해야 한다. 타일 식별자, 압축 복셀 지도(Compressed Voxel Map), 키프레임, 디스크립터(Descriptor), 변화 요약(Change Summary), 선택된 진단 관측값 등을 효율적으로 교환할 수 있다. 전체 해상도의 포인트 클라우드는 지도 재구성이나 문제 분석에 필요한 경우에만 업로드할 수 있다. 이를 통해 대역폭 요구량을 줄이고 Wi-Fi, 사설 5G(Private 5G) 또는 혼합 네트워크에서 대규모 플릿 운용을 현실화할 수 있다.

프리페칭(Prefetching)은 로봇이 지도 영역 사이를 이동할 때 운용 효율을 향상시킨다. 플릿 또는 내비게이션 시스템은 계획 경로의 상당 부분을 이미 알고 있으므로 AMR이 해당 영역에 도달하기 전에 필요한 타일을 다운로드할 수 있다. 차량 속도, 경로 불확실성, 저장장치 지연, 네트워크 품질에 따라 어느 정도 앞의 데이터를 캐시할 것인지 결정한다. 경로가 예상하지 못하게 변경되면 새로운 경로 데이터가 준비될 때까지 인접 타일을 보조 버퍼(Secondary Buffer)로 사용할 수 있다.

네트워크 복원력(Network Resilience)을 위해서는 로컬 캐싱(Local Caching)이 필요하다. 각 로봇은 짧은 통신 장애 중에도 운용을 지속할 수 있을 만큼 충분한 지도 정보를 유지해야 한다. 자주 사용하는 타일, 현재 임무 경로, 복구 영역, 주변 대체 경로 등을 차량 내부에 저장할 수 있다. 시스템은 네트워크 연결 손실과 위치 추정 손실을 구분해야 한다. 유효한 캐시 지도를 가진 로봇은 중앙 서버에 일시적으로 연결할 수 없더라도 정상적인 위치 추정을 유지할 수 있다.

지도 공유 아키텍처는 위치 추정 상태 보고(Localization-Health Reporting)도 지원해야 한다. 로봇은 스캔 정합 잔차, 지도 중첩도, 공분산, 퇴화 지표(Degeneracy Indicator), GNSS 불일치, 재위치 추정 빈도(Relocalization Frequency)와 같은 압축된 통계를 전송할 수 있다. 이러한 지표를 공간적으로 집계하면 여러 로봇이 공통적으로 위치 추정에 어려움을 겪는 영역을 식별할 수 있다. 플릿 전체의 증거를 활용하면 개별 로봇 궤적만으로는 명확하지 않은 지도 문제 영역을 발견할 수 있다.

이러한 위치 추정 품질 지도(Localization-Quality Map)는 임무 계획에도 영향을 줄 수 있다. 특정 복도에서 지속적으로 낮은 LiDAR 관측 가능성이 나타나는 경우 운용 조건이 허용한다면 더 강한 기하 제약을 제공하는 경로를 우선적으로 선택할 수 있다. 약한 영역에 진입하는 로봇은 속도를 낮추고 휠 오도메트리 또는 관성 추정에 대한 의존도를 높이거나 지도화된 랜드마크를 탐색할 수 있다. 따라서 공유 지도는 단순한 기하 구조뿐 아니라 예상 위치 추정 신뢰도(Expected Localization Confidence)도 표현할 수 있다.

이기종 플릿(Heterogeneous Fleet)은 로봇마다 서로 다른 LiDAR, 장착 높이, 시야각, IMU 또는 차량 크기를 사용할 수 있기 때문에 추가적인 문제를 발생시킨다. 공통 지도(Common Map)는 특정 센서 구성에서만 보이는 특징이 아니라 여러 플랫폼에서 관측 가능한 기하 구조를 포함해야 한다. 필요한 경우 센서별 위치 추정 계층(Sensor-Specific Localization Layer)을 별도로 생성할 수 있지만 모든 플랫폼은 동일한 전역 좌표계에 정렬되어야 한다.

이러한 플릿에서는 보정 메타데이터(Calibration Metadata)가 중요하다. 특정 센서 구성으로 생성된 지도는 다른 플랫폼에 서로 다른 방식으로 영향을 주는 체계적인 특성을 포함할 수 있다. 로봇이 지도 관측값을 제공할 때 센서 모델, 외부 파라미터 보정 버전(Extrinsic Calibration Version), 관련 전처리 설정을 함께 보고해야 한다. 이러한 정보는 하드웨어 유지보수 이후 의심스러운 지도 업데이트나 위치 추정 성능 저하가 발생했을 때 추적성(Traceability)을 제공한다.

다중 로봇 루프 폐쇄(Multi-Robot Loop Closure)는 서로 다른 AMR이 생성한 궤적을 연결할 수 있다. 두 로봇이 동일한 특징적인 장소를 관측하면 검증된 로봇 간 제약(Inter-Robot Constraint)을 통해 각 로봇의 궤적을 정렬하고 전역 지도 일관성을 향상시킬 수 있다. 잘못된 로봇 간 연관은 여러 세션을 동시에 변형시킬 수 있으므로 이러한 제약은 보수적으로 검증해야 한다. 기하학적 검증, 타임스탬프, 경로 맥락(Route Context), 전역 기준 정보를 함께 활용해야 한다.

전역 포즈 그래프 최적화(Global Pose-Graph Optimization)는 연속 오도메트리, 로봇 내부 루프 폐쇄(Intra-Robot Loop Closure), 로봇 간 일치 관계, GNSS 또는 RTK 측정값, 측량 랜드마크를 통합할 수 있다. 최적화된 결과는 전체 플릿을 위한 공통 공간 해(Common Spatial Solution)를 형성한다. 최적화 과정에서 과거 서브맵 위치가 변경될 수 있으므로 실제 운용 시스템에서는 로봇이 임무를 수행하는 동안 좌표를 조용히 변경하기보다 새로운 제어된 지도 버전으로 결과를 배포해야 한다.

플릿 지도 서버(Fleet Map Server)는 여러 논리적 서비스로 구성할 수 있다. 영구 저장소(Persistent Repository)는 검증된 지도와 과거 버전을 저장하고 공간 인덱스(Spatial Index)는 위치 및 경로를 타일 식별자와 연결한다. 배포 서비스(Distribution Service)는 지도 데이터를 로봇에 전달하고 업데이트 파이프라인(Update Pipeline)은 후보 관측값을 처리한다. 모니터링 서비스는 위치 추정 품질과 배포 상태를 추적한다. 이러한 기능은 지연 시간, 보안, 현장 요구사항에 따라 중앙 또는 온프레미스에서 실행할 수 있다.

공유 지도는 실제 로봇의 물리적 움직임에 직접 영향을 주기 때문에 보안(Security)과 무결성(Integrity)이 필수적이다. 로봇은 지도 데이터의 출처를 인증하고 다운로드한 데이터를 검증하며 손상되거나 승인되지 않은 버전을 거부해야 한다. 접근 제어(Access Control)를 통해 일반적인 위치 추정 클라이언트와 지도 업데이트를 제안, 승인 또는 배포할 수 있는 시스템을 분리해야 한다. 감사 기록(Audit Record)을 통해 어떤 로봇이 데이터를 제공했는지, 어떤 과정에서 검증되었는지, 언제 해당 버전이 실제 운용에 투입되었는지를 보존할 수 있다.

장애 모드(Failure Mode)는 명시적으로 설계해야 한다. 필요한 타일을 다운로드할 수 없는 경우 로봇은 캐시된 기하 정보를 사용하거나 일시적으로 LiDAR-관성 오도메트리를 이용하여 운행을 지속하거나 대체 경로를 요청하거나 속도를 낮추거나 안전한 위치에 정지할 수 있다. 지도 버전 불일치가 감지되면 시스템은 호환되지 않는 타일을 조용히 혼합해서는 안 된다. 불확실한 위치 추정 상태에서 임무 속도를 유지하는 것보다 예측 가능한 성능 저하 동작(Predictable Degraded Behavior)이 바람직하다.

플릿 지도 공유는 점진적인 지도 개선(Progressive Map Improvement)도 가능하게 한다. 초기 매핑 과정에서 거의 방문하지 않았던 영역도 로봇이 정상적인 임무를 수행하면서 점차 더 많은 관측 데이터를 확보할 수 있다. 여러 관점에서 얻어진 안정적인 관측값은 기하학적 완성도(Geometric Completeness)를 향상시키며 반복되는 궤적을 통해 어떤 구조물이 신뢰할 수 있는 위치 추정 기준인지 판단할 수 있다. 지도 업데이트가 필터링되고 제어된다는 조건에서 실제 운용 플릿은 하나의 분산 센싱 네트워크(Distributed Sensing Network)로 기능할 수 있다.

야외 자율이동로봇(Outdoor AMR)의 경우 GNSS/RTK를 이용하여 공유 LiDAR 지도를 지리 좌표(Geographic Coordinate)에 연결하고 서로 멀리 떨어진 영역 사이의 정렬을 단순화할 수 있다. 건물 주변, 식생 아래, 터널 내부 및 기타 GNSS 품질이 저하되는 영역에서는 로컬 LiDAR-관성 위치 추정이 계속 중요하다. 따라서 공유 아키텍처는 지리적 앵커링(Geographic Anchoring)과 세밀한 로컬 기하 구조를 결합하면서 하나의 플릿 전역 임무 좌표계(Fleet-Wide Mission Coordinate System)를 유지할 수 있다.

핵심 공학 원칙은 지도 공유가 모든 로봇에게 하나의 공통 포인트 클라우드를 자유롭게 수정하도록 허용하는 것과 동일하지 않다는 점이다. 신뢰성 높은 플릿 운용을 위해서는 로컬 추정(Local Estimation), 후보 관측(Candidate Observation), 검증된 지도 제품(Validated Map Product), 버전 제어 기반 배포(Version-Controlled Deployment), 롤백을 서로 분리해야 한다. 이를 통해 일시적인 환경 변화나 개별 로봇의 위치 추정 실패가 즉시 전체 차량에 전파되는 것을 방지할 수 있다.

성숙한 AMR 플릿은 공유 LiDAR 지도를 관리되는 인프라(Managed Infrastructure)로 취급한다. 로봇은 공간 정보를 사용하고 새로운 관측 증거를 제공하며 위치 추정 품질을 보고하고 로컬에 캐시된 검증 버전을 기반으로 운용된다. 중앙 서비스는 관측값을 통합하고 환경 변화를 검출하며 전역 일관성을 최적화하고 제어된 지도 개정판을 배포한다. 이러한 아키텍처를 통해 LiDAR SLAM은 단일 로봇의 자율 운용에서 수백 대의 이동 플랫폼에 걸친 협력적 위치 추정(Coordinated Localization)으로 확장될 수 있다.
