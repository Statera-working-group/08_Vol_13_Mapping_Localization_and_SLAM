**Volume 13. Mapping Localization and SLAM**

# Chapter 01. Localization Fundamentals

## 01.01. Localization Problem State Estimation and Uncertainty

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

위치추정(Localization)은 로봇이 환경에서 동작하는 동안 기준 좌표계(reference frame)를 기준으로 자신의 상태(state)를 추정하는 과정이다. 이동 로봇(mobile robot)의 경우 이러한 상태에는 일반적으로 위치(position)와 자세(orientation)가 포함되지만, 내비게이션 시스템(navigation system)에 필요한 속도(velocity), 가속도(acceleration), 센서 바이어스(sensor bias), 휠 슬립(wheel slip) 변수 또는 기타 물리량도 포함될 수 있다. 따라서 위치추정은 단순히 좌표를 결정하는 것을 넘어 불완전하고 잡음이 포함된 관측값(observation)으로부터 로봇의 상태를 지속적으로 복원하는 과정이다.

위치추정 문제(localization problem)가 발생하는 근본적인 이유는 로봇의 실제 물리적 상태(true physical state)를 일반적으로 직접적이고 완벽하게 측정할 수 없기 때문이다. 엔코더(encoder)는 휠의 움직임을 측정하고, 관성 센서(inertial sensor)는 각속도(angular velocity)와 가속도(acceleration)를 측정하며, 카메라(camera)는 시각 특징(visual feature)을 관측하고, 라이다(LiDAR)는 주변 환경의 기하 구조(geometry)를 측정하며, 위성항법시스템(GNSS)은 전역 위치(global position) 정보를 제공한다. 각각의 측정값은 상태의 일부만 표현하며 잡음(noise), 바이어스(bias), 지연(delay), 모호성(ambiguity), 환경적 간섭(environmental interference)을 포함하므로 정확한 로봇 자세(pose)를 직접 복원하기 어렵다.

유용한 수학적 표현에서는 시간 t에서의 로봇 상태를 상태 벡터(state vector)로 나타낸다. 예를 들어 평면 운동(planar motion)의 경우 xₜ = [x, y, θ]로 표현할 수 있으며, 보다 복잡한 3차원 시스템에서는 위치(position), 자세(orientation), 속도(velocity), 센서 바이어스(sensor bias)를 포함하여 xₜ = [p, q, v, b]와 같이 나타낼 수 있다. 위치추정에서는 xₜ가 정확히 알려져 있다고 가정하지 않고 가능한 상태에 대한 확률분포(probability distribution)를 추정한다. 이 분포는 가장 가능성이 높은 상태뿐만 아니라 해당 추정값과 관련된 불확실성(uncertainty)도 함께 표현한다.

상태 추정(state estimation)은 근본적으로 재귀적 추론 문제(recursive inference problem)이다. 로봇은 자신의 상태에 대한 사전 믿음(prior belief)에서 시작하여 명령된 운동(commanded motion)이나 측정된 동역학(measured dynamics)에 따라 이 믿음이 어떻게 변화하는지를 예측한다. 새로운 센서 관측(sensor observation)이 입력되면 가능한 상태와 관측값 사이의 일치 정도를 이용하여 예측된 믿음을 보정한다. 이러한 예측과 보정(prediction-and-correction) 과정은 로봇이 이동하는 동안 반복적으로 수행되며, 이를 통해 로봇의 상태 추정값이 지속적으로 갱신된다.

운동 모델(motion model)은 제어 입력(control input)과 물리적 동역학(physical dynamics)에 따라 이전 상태가 새로운 상태로 어떻게 변화하는지를 설명한다. 자율이동로봇(AMR)의 경우 입력에는 휠 속도(wheel velocity), 조향 명령(steering command), 또는 측정된 관성 운동(inertial motion)이 포함될 수 있다. 실제 운동은 휠 슬립(wheel slip), 불균일한 지형(uneven terrain), 액추에이터 오차(actuator error), 컴플라이언스(compliance), 외란(disturbance) 등의 영향으로 수학적 모델을 완벽하게 따르지 않는다. 따라서 예측 단계(prediction step)에서는 일반적으로 로봇 상태가 시간에 따라 전파되면서 불확실성이 증가한다.

관측 모델(observation model)은 숨겨진 로봇 상태(hidden robot state)와 센서가 생성할 것으로 예상되는 측정값 사이의 관계를 정의한다. 라이다 위치추정(LiDAR localization) 시스템은 후보 자세(candidate pose)로부터 기하학적 거리(geometric range)를 예측할 수 있으며, 시각 기반 시스템(visual system)은 영상에서 랜드마크(landmark)의 위치를 예측할 수 있다. 위성항법시스템(GNSS)은 지리적 위치(geographic position)를 위성 기반 관측값과 연결하고, 휠 엔코더(wheel encoder)는 상대 운동(relative motion)을 제약한다. 위치추정 시스템은 실제 측정값과 이러한 예측값을 비교하여 어떤 후보 상태가 더 가능성이 높은지를 판단한다.

불확실성(uncertainty)은 단순히 위치추정 과정에서 발생하는 바람직하지 않은 부작용이 아니라 상태 추정 문제를 구성하는 명시적인 요소이다. 로봇은 현재 자세를 얼마나 신뢰할 수 있는지뿐만 아니라 불확실성이 각각의 상태 변수에 어떻게 분포되어 있는지도 표현해야 한다. 가우시안 추정기(Gaussian estimator)에서는 이러한 정보를 일반적으로 공분산 행렬(covariance matrix)로 표현한다. 큰 공분산(covariance)은 낮은 신뢰도를 의미하며, 상태 변수 사이의 상관관계(correlation)는 한 변수의 불확실성이 다른 변수의 불확실성과 어떻게 연관되는지를 나타낸다.

불확실성은 근본적으로 서로 다른 여러 원인에서 발생한다. 측정 잡음(measurement noise)은 무작위 변동(random variation)을 발생시키고, 체계적인 센서 바이어스(systematic sensor bias)는 지속적인 오차를 만든다. 불완전한 보정(imperfect calibration)은 센서 간 관계를 왜곡하며, 불확실한 동역학(uncertain dynamics)은 상태 전파(state propagation) 과정에서 오차를 발생시킨다. 또한 조명 변화(changing illumination), 반사 표면(reflective surface), 반복적인 기하 구조(repetitive geometry), GNSS 다중경로(GNSS multipath), 먼지(dust), 비(rain), 진동(vibration), 휠 슬립(wheel slip)과 같은 환경적 영향도 추가적인 불확실성을 발생시킨다. 실용적인 위치추정 아키텍처(localization architecture)는 이러한 영향을 동시에 고려해야 한다.

위치추정 오차(localization error)는 일반적으로 상대 드리프트(relative drift)와 절대 오차(absolute error)의 두 가지 주요 형태로 구분할 수 있다. 오도메트리(odometry)와 관성 적분(inertial integration)은 단기간의 운동을 부드럽게 추정할 수 있지만 작은 오차가 이동 거리와 시간이 증가함에 따라 누적된다. 위성항법시스템(GNSS), 지도화된 랜드마크(mapped landmark), 인공 표식(fiducial), 기존에 구축된 지도(map)와 같은 전역 기준(global reference)은 이러한 드리프트를 제한할 수 있다. 따라서 강건한 시스템(robust system)은 국부적으로 정확한 상대 측정(relative measurement)과 지속적인 전역 기준과의 일관성을 회복할 수 있는 관측 정보를 결합한다.

위치추정은 좌표계(coordinate frame)의 선택에도 크게 의존한다. 로봇은 동시에 바디 좌표계(body frame), 오도메트리 좌표계(odometry frame), 로컬 지도 좌표계(local map frame), 전역 기준 좌표계(globally referenced frame)를 사용할 수 있다. 이러한 좌표계 사이의 변환(transformation)은 센서 측정값, 지도, 궤적(trajectory), 제어 명령(control command)이 어떻게 해석되는지를 결정한다. 특히 다중 센서 시스템(multi-sensor system)에서는 공간적 또는 시간적 정렬 오류가 추정기(estimator)에서 실제 물리적 운동처럼 인식될 수 있으므로 일관된 타임스탬프(timestamp)와 좌표 변환(frame transformation)을 유지하는 것이 중요하다.

관측 가능성(observability)은 사용 가능한 측정값이 특정 상태 변수를 추정하기에 충분한 정보를 포함하는지를 결정한다. 로봇이 특징이 거의 없는 긴 복도(featureless corridor)를 이동하는 경우 횡방향 위치(lateral position)는 정확하게 제한할 수 있지만 복도의 진행 방향에 대한 위치는 높은 불확실성을 가질 수 있다. 시각 위치추정(visual localization)은 텍스처가 부족한 환경(textureless environment)에서 성능이 저하될 수 있으며, 라이다 위치추정(LiDAR localization)은 기하 구조가 반복되는 공간에서 모호해질 수 있다. 센서를 추가하는 것만으로 이러한 문제가 자동으로 해결되는 것은 아니며, 추가된 센서의 측정값이 상호 보완적인 관측 가능 정보(complementary observable information)를 제공해야 한다.

서로 다른 추정 알고리즘(estimation algorithm)은 불확실성을 서로 다른 방식으로 표현한다. 칼만 필터 계열(Kalman-filter family)은 통계적 모멘트(statistical moment)를 이용하여 상태 분포를 근사하며 불확실성이 비교적 안정적으로 표현될 수 있는 환경에서 효율적이다. 파티클 필터(particle filter)는 가중 샘플(weighted sample)을 사용하여 여러 가설(multiple hypothesis)을 표현하기 때문에 여러 자세가 동시에 가능할 때 다봉 분포(multimodal belief)를 유지할 수 있다. 최적화 기반 추정기(optimization-based estimator)는 상태와 측정값 사이의 제약조건(constraint)을 구성하고 누적된 정보를 가장 잘 만족하는 궤적을 계산한다.

베이지안 관점(Bayesian viewpoint)은 이러한 다양한 접근 방법에 공통적인 개념적 기반을 제공한다. 예측(prediction)은 운동 모델을 적용하여 이전의 믿음(belief)을 다음 상태에 대한 사전 확률분포(prior distribution)로 변환한다. 측정 갱신(measurement update)은 이러한 사전 정보에 새로운 관측값의 우도(likelihood)를 결합하여 사후 믿음(posterior belief)을 생성한다. 따라서 위치추정은 운동에 의해 불확실성이 확산되고 정보성이 높은 관측에 의해 불확실성이 감소하거나 재구성되는 반복적인 베이지안 추론(Bayesian inference) 과정으로 이해할 수 있다.

개별 센싱 방식(sensing modality)은 서로 보완적인 장점과 고장 형태(failure mode)를 가지므로 센서 융합(sensor fusion)이 필요하다. 엔코더(encoder)는 저비용으로 높은 주기의 상대 운동 정보를 제공하지만 슬립(slip)의 영향을 받는다. 관성측정장치(IMU)는 빠른 동적 움직임을 측정하지만 적분 오차(integration error)가 누적된다. 카메라(camera)는 풍부한 의미적·기하학적 정보를 제공하지만 가시성과 조명 조건에 영향을 받는다. 라이다(LiDAR)는 신뢰성 높은 기하 구조를 제공하며, 위성항법시스템(GNSS)은 야외에서 전역 기준을 제공하지만 건물, 식생, 터널 또는 전파 간섭 환경에서는 신뢰성이 저하될 수 있다.

잘 설계된 위치추정 시스템(localization system)은 이러한 측정값을 단순히 평균하지 않는다. 대신 불확실성, 시간 정보, 측정값의 일관성, 예상되는 센서 특성을 기반으로 각 측정값을 평가한다. 현재 상태와 크게 불일치하는 측정값은 실제 상태를 교정할 수 있는 중요한 정보일 수도 있지만 센서 고장이나 환경적 외란으로 발생한 이상치(outlier)일 수도 있다. 혁신 검정(innovation test), 강건 손실 함수(robust loss function), 게이팅(gating), 중복성(redundancy), 고장 검출(fault detection)을 이용하면 단일 오류 관측값이 전체 상태 추정값을 불안정하게 만드는 것을 방지할 수 있다.

따라서 시간 동기화(time synchronization)는 상태 추정 정확도(state-estimation accuracy)와 밀접하게 연결된다. 서로 다른 주기로 동작하는 센서는 서로 다른 물리적 시점에서 로봇을 관측하며, 로봇이 빠르게 이동하거나 회전할 경우 작은 타임스탬프 오차(timestamp error)도 상당한 공간 오차(spatial error)를 발생시킬 수 있다. 차량 속도, 센서 수, 위치추정 정밀도 요구사항이 증가할수록 하드웨어 타임스탬프(hardware timestamp), 동기화된 클록(synchronized clock), 결정론적 데이터 획득 파이프라인(deterministic acquisition pipeline), 정확한 지연 모델(latency model)의 중요성이 증가한다.

초기화(initialization)는 위치추정에서 또 하나의 중요한 문제이다. 초기 자세(initial pose)를 대략적으로 알고 있는 경우 추정기는 집중된 사전 분포(concentrated prior distribution)에서 시작하여 빠르게 상태를 정교화할 수 있다. 초기 자세를 알 수 없는 경우에는 시스템이 여러 가능한 가설을 고려하여 전역 위치추정(global localization)을 수행해야 한다. 기존에 위치를 정상적으로 추정하던 로봇도 심각한 센서 성능 저하(sensor degradation)나 지도 불일치(map inconsistency) 이후 위치를 상실할 수 있으며, 이러한 경우 일반적인 점진적 자세 추적(incremental pose tracking)이 아니라 재위치추정(relocalization)이 필요하다.

따라서 위치추정 품질(localization quality)은 하나의 위치 오차(position error) 수치만으로 평가해서는 안 된다. 중요한 평가 지표에는 병진 오차(translational error), 회전 오차(rotational error), 공분산 일관성(covariance consistency), 이동 거리당 드리프트(drift per traveled distance), 재위치추정 성공률(relocalization success), 초기화 시간(initialization time), 갱신 지연(update latency), 고장 빈도(failure frequency), 센싱 성능이 저하된 환경에서의 강건성(robustness)이 포함된다. 자율 로봇에서는 의도된 운용 영역(operating domain) 전체에서 추정된 상태가 계획(planning)과 제어(control)에 충분할 정도로 정확하고 신속하며 신뢰할 수 있는지가 실질적으로 중요하다.

상태 불확실성(state uncertainty)은 하위 단계의 자율주행 기능(downstream autonomy)에도 직접적인 영향을 미친다. 불확실한 자세를 정확한 값으로 가정하는 경로 계획기(planner)는 장애물과의 안전 여유(obstacle clearance)가 부족한 궤적을 선택할 수 있으며, 제어기(controller)는 위치추정 잡음에 의해 발생한 겉보기 추종 오차(apparent tracking error)에 잘못 반응할 수 있다. 고급 시스템에서는 위치추정 신뢰도(localization confidence)를 경로 계획, 속도 제어(speed control), 안전 여유(safety margin), 임무 관리(mission management)에 전달할 수 있다. 이를 통해 로봇은 신뢰도가 낮아질 때 속도를 줄이고, 더 좋은 관측 정보를 확보하거나, 위치추정 소스를 전환하고, 필요한 경우 안전하게 정지할 수 있다.

야외 자율이동로봇(Outdoor AMR)의 위치추정은 포장도로(pavement), 자갈길(gravel), 경사면(slope), 식생(vegetation), 건물 주변, 개방된 하늘(open sky), GNSS 음영 지역(GNSS-denied region) 등 운용 조건이 지속적으로 변화하기 때문에 특히 어렵다. 휠 슬립(wheel slip)은 오도메트리(odometry)의 정확성을 떨어뜨릴 수 있고, 진동(vibration)은 관성 측정값을 저하시킬 수 있으며, 환경의 기하 구조는 지도 기반 제약조건(map constraint)에 불일치를 발생시킬 수 있다. 따라서 신뢰성 높은 실제 운용을 위해서는 하나의 센서에 의존하기보다 국부 운동 추정(local motion estimation), 기하 또는 시각 기반 위치추정(geometric or visual localization), 전역 기준 측정(globally referenced measurement)을 적응적으로 융합해야 한다.

궁극적으로 위치추정(localization)은 하나의 절대적으로 정확한 좌표를 출력하는 메커니즘이 아니라 지속적인 확률적 상태 추정(continuous probabilistic state estimation) 과정으로 다루어야 한다. 모든 자세 추정값(pose estimate)은 모델(model), 관측값(observation), 가정(assumption), 그리고 정량화할 수 있는 불확실성(uncertainty)에 의해 뒷받침된다. 이러한 원칙을 기반으로 자율 로봇을 설계하면 센서 융합(sensor fusion), 동시적 위치추정 및 지도작성(SLAM), 지도 기반 위치추정(map-based localization), GNSS 통합(GNSS integration), 다중 로봇 협업(multi-robot coordination), 안전 인지 내비게이션(safety-aware navigation)으로 확장할 수 있는 기본 토대를 구축할 수 있다.

## 01.02. Coordinate Frames and Transforms World Map Odom Base

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

자율 로봇(autonomous robot)은 좌표계(coordinate frame) 없이 위치와 움직임을 의미 있게 표현할 수 없다. 모든 센서 측정값(sensor measurement), 로봇 자세(robot pose), 지도 특징(map feature), 속도(velocity), 궤적(trajectory)은 특정 기준 좌표계(reference frame)를 기준으로 표현된다. 좌표계는 물리량을 해석하기 위한 원점(origin)과 방향이 정의된 축(axis)을 제공한다. 따라서 위치추정 시스템(localization system)은 로봇이 어디에 있는지를 추정하는 것뿐만 아니라 각각의 추정값이 어떤 기준 좌표계에서 표현되는지를 명확하게 정의해야 한다.

이동 로봇(mobile robot)은 하나의 좌표계만으로 모든 내비게이션 요구사항을 충족할 수 없기 때문에 일반적으로 여러 좌표계를 동시에 사용한다. 월드 좌표계(world frame)는 지속적인 전역 기준(global reference)을 나타내고, 맵 좌표계(map frame)는 구축되거나 사전에 정의된 지도에 대한 위치추정을 나타내며, 오도메트리 좌표계(odom frame)는 국부적으로 연속적인 운동을 제공한다. 베이스 좌표계(base frame)는 로봇 본체에 강체로 고정된다. 센서와 액추에이터(actuator)는 보정된 좌표 변환(calibrated transformation)을 통해 이러한 계층 구조에 연결되는 추가적인 좌표계를 가진다.

월드 좌표계(world frame)는 로봇이 전역적으로 의미 있는 좌표 시스템(global coordinate system)에서 동작해야 할 때 가장 상위 수준의 기준을 제공한다. 이는 시설 좌표계(facility coordinate system), 측량 좌표계(engineering survey frame), 지리적 기준(geographic reference), 또는 지구 고정 좌표계(Earth-fixed convention)에 대응할 수 있다. 야외 시스템에서는 위성항법시스템(GNSS)과 측량된 랜드마크(surveyed landmark)를 이용하여 위치추정을 이 좌표계와 연결할 수 있다. 월드 좌표계는 여러 지도, 로봇, 건물 또는 지리적으로 분리된 운용 영역이 하나의 공통 공간 기준(common spatial reference)을 공유해야 할 때 특히 유용하다.

맵 좌표계(map frame)는 일반적으로 위치추정과 내비게이션에 사용되는 환경 모델(environment model)의 좌표 시스템을 나타낸다. 동시적 위치추정 및 지도작성(SLAM)으로 생성된 점유 지도(occupancy map), 라이다 포인트 클라우드 지도(LiDAR point-cloud map), 시각 랜드마크 지도(visual landmark map), 또는 고정밀 지도(HD map)는 각각 자체적인 맵 좌표계를 정의할 수 있다. 로봇 위치추정은 로봇과 이 지도 사이의 좌표 변환을 결정한다. 명시적인 지리참조(georeferencing)가 수행되지 않았다면 맵 좌표계는 월드 좌표계와 달리 지리 좌표와 직접 대응할 필요가 없다.

오도메트리 좌표계(odom frame)는 로봇 운동에 대한 국부적으로 부드럽고 연속적인 추정값을 제공한다. 일반적으로 휠 오도메트리(wheel odometry), 시각-관성 오도메트리(visual-inertial odometry), 라이다 오도메트리(LiDAR odometry), 또는 융합 추측항법(fused dead reckoning)을 이용하여 생성된다. 이러한 측정값에는 오차가 누적되기 때문에 오도메트리 좌표계는 맵 또는 월드 좌표계에 대해 점진적으로 드리프트(drift)한다. 그러나 전역 위치추정(global localization)의 보정으로 인해 자세 추정값에 불연속적인 변화가 발생할 수 있는 상황에서도 단기간의 로봇 운동을 부드럽게 유지할 수 있다는 중요한 장점을 가진다.

베이스 좌표계(base frame)는 로봇 소프트웨어 아키텍처에서 흔히 베이스 링크(base_link)라고 하며 물리적인 로봇 본체에 강체로 연결된다. 원점(origin)은 기하학적 중심(geometric center), 차축 중심(axle center), 또는 다른 안정적인 기준점과 같은 명확한 기계적 규칙에 따라 선정된다. 경로 계획기(motion planner), 제어기(controller), 충돌 모델(collision model), 센서 좌표 변환(sensor transform)은 이 좌표계를 사용하여 로봇에 상대적인 물리량을 표현한다. 베이스 좌표계의 원점이 변경되면 로봇 운동에 대한 수학적 해석도 달라지므로 일관된 정의가 필수적이다.

이러한 좌표계 사이의 관계는 강체 좌표 변환(rigid-body transformation)의 연속적인 체인(chain)으로 표현할 수 있다. 일반적인 내비게이션 계층 구조는 월드(world) → 맵(map) → 오도메트리(odom) → 베이스(base)의 순서로 구성된다. 따라서 월드 좌표계에서의 로봇 자세는 인접한 좌표계 사이의 변환을 합성하여 계산할 수 있다. 수학적으로 Tᴬ_B가 좌표계 A를 기준으로 좌표계 B를 표현한다면, Tᵂ_B는 좌표계 트리(frame tree)를 따라 필요한 변환을 올바른 순서로 곱하여 구할 수 있다.

강체 좌표 변환(rigid transformation)은 병진(translation)과 회전(rotation)을 모두 포함한다. 평면 이동 로봇(planar mobile robot)에서는 자세를 x, y, 헤딩(heading) θ로 표현할 수 있지만, 3차원 시스템에서는 3차원 병진과 자세 표현(orientation representation)이 필요하다. 회전 행렬(rotation matrix), 오일러 각(Euler angle), 축-각 표현(axis-angle representation), 쿼터니언(quaternion)이 일반적으로 사용된다. 동차 변환 행렬(homogeneous transformation matrix)은 회전과 병진을 하나의 표현으로 결합하여 좌표 변환의 합성(transform composition)을 편리하게 수행할 수 있도록 한다.

좌표 변환 방향(transformation direction)은 항상 주의 깊게 다루어야 한다. 좌표계 A를 기준으로 좌표계 B를 표현하는 변환은 좌표계 B를 기준으로 좌표계 A를 표현하는 변환과 수치적으로 동일하지 않다. 변환 방향을 반대로 적용하려면 역변환(inverse transformation)이 필요하다. 많은 위치추정 및 센서 융합(sensor fusion) 오류는 복잡한 알고리즘 자체가 아니라 좌표계 규칙(frame convention), 변환 방향, 축 방향(axis orientation), 행렬 곱셈 순서(multiplication order)를 잘못 해석하면서 발생한다.

자세 규칙(orientation convention)은 또 다른 잠재적인 불일치의 원인이 된다. 로봇 시스템에서는 오른손 좌표계(right-handed coordinate system)를 사용할 수 있지만 센서나 외부 소프트웨어는 서로 다른 축 규칙을 사용할 수 있다. 차량 좌표계(vehicle frame)는 x축을 전방, y축을 좌측, z축을 상방으로 정의할 수 있지만 카메라는 서로 다른 방향을 갖는 광학 좌표계(optical frame)를 사용할 수 있다. 위성항법시스템(GNSS), 항공우주(aerospace), 컴퓨터 비전(computer vision), 지리정보시스템(GIS)에서도 ENU, NED, 영상 좌표(image coordinate)와 같은 서로 다른 규칙을 사용하므로 명시적인 변환이 필요하다.

정적 좌표 변환(static transform)은 정상적인 운용 중에 변하지 않는 관계를 표현한다. 베이스 좌표계에서 강체로 장착된 라이다(LiDAR), 카메라(camera), GNSS 안테나(GNSS antenna), 관성측정장치(IMU)까지의 변환이 대표적인 예이다. 이러한 변환은 기계 설계(mechanical design) 또는 외부 파라미터 보정(extrinsic calibration)을 통해 결정된다. 작은 오차라도 큰 영향을 미칠 수 있으며, 센서의 각도 보정 오차(angular calibration error)는 측정 거리가 증가할수록 공간 오차를 증가시켜 지도작성(mapping), 위치추정, 다중 센서 융합(multi-sensor fusion)의 성능을 저하시킬 수 있다.

동적 좌표 변환(dynamic transform)은 로봇이 이동하거나 관절형 구성 요소(articulated component)의 상태가 변화함에 따라 달라진다. 오도메트리 좌표계와 베이스 좌표계 사이의 변환은 차량의 움직임에 따라 지속적으로 변화하며, 팬-틸트 카메라(pan-tilt camera), 매니퓰레이터 관절(manipulator joint), 조향 메커니즘(steering mechanism), 서스펜션 시스템(suspension system)도 추가적인 동적 좌표계 관계를 생성할 수 있다. 따라서 좌표 변환 시스템은 고정된 기하학적 관계와 시간에 따라 변화하는 상태를 구분하고, 하위 알고리즘(downstream algorithm)에 적합한 주기로 동적 변환을 갱신해야 한다.

맵 좌표계(map frame)와 오도메트리 좌표계(odom frame)를 분리하면 중요한 아키텍처 문제를 해결할 수 있다. 전역 위치추정 알고리즘(global localization algorithm)은 랜드마크 인식(landmark recognition), 지도 정합(map matching), SLAM 루프 폐쇄(loop closure), GNSS 정보 수신 등을 통해 누적된 드리프트를 주기적으로 보정할 수 있다. 이러한 보정을 국부적으로 적분된 로봇 자세에 직접 적용하면 갑작스러운 점프가 발생할 수 있다. 대신 오도메트리-베이스(odom-to-base) 변환은 연속성을 유지하고, 맵-오도메트리(map-to-odom) 변환이 전역 보정값을 흡수한다.

이러한 분리는 제어(control)와 전역 내비게이션(global navigation)이 서로 다르지만 호환 가능한 공간적 특성을 이용할 수 있도록 한다. 저수준 제어기(low-level controller)는 부드러운 오도메트리 좌표계를 사용하여 안정적인 속도 및 궤적 추종(trajectory tracking)을 수행할 수 있고, 전역 경로 계획기(global planner)는 맵 좌표계에서 로봇의 위치와 경로를 판단할 수 있다. 좌표 변환 시스템은 두 표현을 연결하여 모든 구성 요소가 동일한 기준 특성을 사용하도록 강제하지 않고도 전역 목표(global goal)를 국부적으로 실행 가능한 명령으로 변환할 수 있도록 한다.

월드 좌표계(world frame)를 맵 좌표계 위에 추가하면 또 하나의 공간적 조직 계층을 구성할 수 있다. 각각의 지도는 편리한 로컬 좌표(local coordinate)를 유지하면서 전역 시설 또는 지리적 좌표계와의 관계를 별도로 관리할 수 있다. 이는 캠퍼스(campus), 공장(factory), 항만(port), 물류센터(logistics center), 야외 로봇 플릿(outdoor robot fleet)처럼 여러 개의 로컬 지도를 하나의 운용 환경으로 통합해야 하는 경우 특히 유용하다. 모든 지도를 하나의 거대한 좌표계에서 다시 생성하지 않고도 전역 공간 관계를 유지할 수 있다.

다중 로봇 시스템(multi-robot system)에서는 좌표계 관리(frame management)가 더욱 중요해진다. 각각의 로봇은 자체적인 오도메트리 좌표계와 베이스 좌표계를 유지하면서 공통의 맵 또는 월드 좌표계를 공유할 수 있다. 협업 지도작성(collaborative mapping), 플릿 조정(fleet coordination), 작업 할당(task allocation), 충돌 회피(collision avoidance)를 수행하려면 한 로봇이 생성한 정보를 다른 로봇이 올바르게 해석할 수 있도록 좌표 변환이 제공되어야 한다. 고유한 좌표계 명명(unique frame naming), 동기화된 클록(synchronized clock), 공유 지리참조(shared georeferencing), 일관된 보정(consistent calibration)은 확장 가능한 플릿 자율성(fleet autonomy)을 위한 핵심 인프라가 된다.

좌표 변환(coordinate transform)은 시간과도 연결되어야 한다. 시간 t에서 획득된 센서 측정값은 가장 최근의 좌표 변환이 아니라 동일한 시간에 해당하는 로봇 자세와 센서 구성(sensor configuration)을 사용하여 변환해야 한다. 라이다 스캔(LiDAR scan), 카메라 영상(camera image), IMU 샘플이 서로 다른 시점의 좌표 변환과 결합되면 로봇의 실제 움직임이 겉보기 공간 왜곡(apparent spatial distortion)을 발생시킨다. 이러한 문제는 차량 속도, 회전 속도(rotational rate), 처리 지연(processing latency)이 증가할수록 더욱 심각해진다.

따라서 비동기 로봇 시스템(asynchronous robotic system)에서는 좌표 변환 버퍼링(transform buffering)과 보간(interpolation)이 필수적이다. 센서는 서로 다른 주파수로 동작하고 네트워크 패킷(network packet)은 가변적인 지연을 경험하며, 위치추정 결과는 해당 추정에 사용된 관측값보다 늦게 도착할 수 있다. 좌표 변환 프레임워크(transform framework)는 시간에 따라 인덱싱된 좌표계 관계의 이력(history)을 유지하고 각 측정 타임스탬프에 적절한 변환을 검색하거나 보간할 수 있다. 정확한 시간 정렬(temporal alignment)은 정확한 공간 보정(spatial calibration)만큼 중요하다.

좌표 변환을 해석할 때는 불확실성(uncertainty)도 고려해야 한다. 정밀하게 보정된 강체 센서 마운트(rigid sensor mount)와 같은 일부 변환은 거의 고정된 것으로 취급할 수 있지만, 위치추정으로 계산되는 변환에는 상당한 통계적 불확실성(statistical uncertainty)이 포함될 수 있다. 환경 특징이 사라지거나 GNSS 품질이 저하되면 맵-오도메트리(map-to-odom) 관계의 불확실성이 증가할 수 있다. 좌표계 트리는 기하학적 관계를 정의하지만 추정 시스템은 이러한 관계와 연관된 신뢰도(confidence), 공분산(covariance), 기타 불확실성 정보를 별도로 유지해야 한다.

좌표계 관리 오류(frame management error)는 전체 자율주행 스택(autonomy stack)으로 전파되는 경우가 많다. 잘못된 센서 외부 파라미터 변환(sensor extrinsic transform)은 지도를 왜곡하고 잘못된 장애물 위치(false obstacle position)를 생성하며 위치추정에 바이어스를 발생시키고 궁극적으로 경로 계획과 제어에 영향을 줄 수 있다. 반대로 적용된 좌표 변환은 객체를 로봇 뒤쪽에 배치할 수 있으며, 잘못된 타임스탬프는 측정값에 인위적인 회전이나 병진을 발생시킬 수 있다. 따라서 좌표계 오류는 실제 알고리즘이 정상적으로 작동하고 있음에도 인지(perception), 위치추정, 지도작성, 제어기 고장처럼 나타날 수 있다.

실제 검증(practical validation)에서는 전체 내비게이션 시스템을 신뢰하기 전에 각각의 좌표 변환을 독립적으로 확인해야 한다. 엔지니어는 센서 데이터가 실제 물리적 기하 구조와 정렬되는지, 정지한 로봇에서 좌표 변환이 안정적으로 유지되는지, 전진 운동이 예상된 축 방향과 일치하는지, 지도 보정(map correction)이 국부 오도메트리의 연속성을 유지하는지를 확인할 수 있다. 알려진 랜드마크와 측량된 기준점(surveyed reference point)을 이용하면 병진, 회전, 스케일(scale), 시간 정렬 오류를 추가로 검출할 수 있다.

야외 자율이동로봇(Outdoor AMR)에서는 월드-맵-오도메트리-베이스(world--map--odom--base) 계층 구조가 GNSS, 관성항법(inertial navigation), 휠 오도메트리, 라이다 위치추정, 지도 기반 내비게이션(map-based navigation)을 결합하기 위한 강력한 기반을 제공한다. GNSS는 시스템을 지리적 또는 측량 기반 월드 좌표계에 고정할 수 있고, 로컬 지도는 정밀한 환경 구조를 제공하며, 오도메트리는 부드러운 단기 운동을 유지하고, 베이스 좌표계는 차량 제어를 지원한다. 이러한 계층 구조는 서로 다른 오차 특성을 분리하면서도 각 좌표계 사이의 일관된 공간 관계를 유지한다.

궁극적으로 좌표계(coordinate frame)는 자율 로봇의 구성 요소들이 서로 통신하기 위한 공간적 언어(spatial language)를 형성한다. 월드(world), 맵(map), 오도메트리(odom), 베이스(base) 좌표계는 동일한 자세를 중복해서 표현하는 것이 아니라 각각 전역 일관성(global consistency), 지도 기반 위치추정(map localization), 국부 연속성(local continuity), 로봇 상대 연산(robot-relative computation)에 적합한 서로 다른 기준을 제공한다. 올바르게 설계된 좌표 변환, 보정(calibration), 시간 관리(timing), 좌표계 규칙(frame convention)은 신뢰성 높은 위치추정, SLAM, 센서 융합, 경로 계획, 제어, 다중 로봇 자율성(multi-robot autonomy)을 구현하기 위한 기하학적 기반을 제공한다.

## 01.03. Dead Reckoning Odometry Accumulation and Drift

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

추측항법(Dead Reckoning)은 외부의 절대 기준(absolute reference)에 지속적으로 의존하지 않고, 이전에 알고 있던 상태로부터 이동량을 적분하여 로봇의 현재 상태를 추정하는 과정이다. 초기 자세(initial pose)에서 시작하여 로봇은 증분 병진(incremental translation)과 회전(rotation)을 측정하고 위치와 자세를 반복적으로 갱신한다. 휠 엔코더(wheel encoder), 관성 센서(inertial sensor), 시각 오도메트리(visual odometry), 라이다 오도메트리(LiDAR odometry), 그리고 이들을 조합한 센서 시스템을 통해 이러한 상대 운동(relative motion) 정보를 얻을 수 있다.

추측항법의 근본적인 장점은 전역 기준(global reference)을 사용할 수 없는 상황에서도 연속적인 국부 운동 추정(local motion estimate)을 제공한다는 것이다. 건물, 터널, 창고, 지하시설 또는 GNSS 음영 야외 지역(GNSS-denied outdoor region)에서도 로봇은 시작 지점을 기준으로 자신이 어떻게 이동했는지를 추정할 수 있다. 높은 주기로 측정값을 생성할 수 있기 때문에 추측항법은 제어(control), 단기 궤적 추종(short-term trajectory tracking), 운동 보상(motion compensation), 느린 전역 관측 사이의 위치추정 유지에 특히 유용하다.

오도메트리(Odometry)는 온보드 센싱(onboard sensing)을 통해 로봇의 증분 운동을 추정하는 추측항법의 실용적인 구현 방법이다. 휠 기반 이동 로봇에서는 엔코더 측정값이 일정한 샘플링 구간 동안의 휠 회전을 나타낸다. 휠 반경(wheel radius), 차축 형상(axle geometry), 차량 운동학(vehicle kinematics)을 이용하면 이러한 회전량을 병진 및 회전 변위로 변환할 수 있다. 계산된 증분값을 시간에 따라 적분하면 오도메트리 좌표계(odometry coordinate frame)에서 지속적으로 변화하는 로봇 자세 추정값을 얻을 수 있다.

차동구동 로봇(differential-drive robot)에서는 좌우 휠의 이동 거리 차이가 헤딩(heading)의 변화를 결정하고, 두 휠 이동 거리의 평균은 대략적인 전진 변위를 결정한다. 애커먼 조향(Ackermann-steered), 전방향(omnidirectional), 스키드 스티어(skid-steer), 무한궤도(tracked), 다축 차량(multi-axle vehicle)은 각각 서로 다른 운동학적 관계가 필요하다. 그러나 기본 원리는 동일하며, 국부적으로 측정된 액추에이터(actuator) 또는 차체 운동을 로봇 자세의 증분 변화로 변환하고 이를 각 시간 단계마다 누적한다.

자세 누적(pose accumulation)은 본질적으로 재귀적(recursive)이다. 시간 k에서 추정된 자세를 알고 있다면 k와 k+1 사이에서 측정된 운동을 현재 자세 방향에 따라 변환하여 이전 상태에 추가한다. 따라서 다음 추정값은 그 이전의 모든 추정값에 의존한다. 이 특성은 하나의 단계에서 발생한 오차가 이후 운동을 적분하는 기준의 일부가 된다는 것을 의미하며, 작은 국부 오차(local error)가 긴 궤적에서 상당한 전역 오차(global error)로 성장할 수 있다.

추측항법 오차(dead-reckoning error)는 개념적으로 체계적 오차(systematic error)와 확률적 오차(stochastic error)로 구분할 수 있다. 체계적 오차는 잘못된 휠 직경, 좌우 휠 반경 차이, 부정확한 윤거(track width), 조향 보정 오차(steering calibration error), 센서 스케일 계수 오차(sensor scale-factor error)와 같이 지속적인 모델의 불완전성에서 발생한다. 이러한 오차는 반복적으로 일정하게 발생하기 때문에 이동 거리나 헤딩에 예측 가능한 바이어스(bias)를 생성할 수 있으며, 정밀한 기계 측정과 보정을 통해 상당 부분 감소시킬 수 있다.

확률적 오차(stochastic error)는 고정된 보정 파라미터(calibration parameter)만으로 표현하기 어려운 영향에 의해 발생한다. 휠 슬립(wheel slip), 노면 불규칙(surface irregularity), 진동(vibration), 타이어 변형(tire deformation), 느슨한 지형(loose terrain), 충돌(collision), 액추에이터 외란(actuator disturbance), 급격한 페이로드(payload) 변화는 측정된 휠 운동과 실제 차량 변위 사이의 관계를 변화시킬 수 있다. 이러한 오차는 운용 조건에 따라 달라지며, 동일한 명령이라도 환경에 따라 서로 다른 실제 운동을 발생시킬 수 있기 때문에 보상이 더욱 어렵다.

자세 방향 오차(orientation error)는 이후의 위치 추정에 영향을 미치기 때문에 특히 중요하다. 작은 헤딩 오차(heading error)는 초기에는 미세한 각도 차이만 발생시키지만, 이후의 모든 전진 변위가 약간 잘못된 방향으로 투영된다. 장거리 이동에서는 이것이 상당한 횡방향 위치 오차(lateral position error)로 증가한다. 따라서 회전 추정(rotational estimation)의 정확도를 향상시키는 것은 이동 로봇의 장기적인 추측항법 정확도 향상에 매우 큰 영향을 미친다.

이러한 오차의 누적은 일반적으로 드리프트(drift)라고 표현한다. 드리프트는 증분 측정값이 국부적으로 부드럽고 합리적으로 보이더라도 추정 궤적(estimated trajectory)이 실제 로봇 궤적(true trajectory)으로부터 점차 벗어나는 현상을 의미한다. 순간적인 무작위 잡음(random instantaneous noise)과 달리 드리프트는 추측항법 자체에 정확한 전역 위치를 복원하는 고유한 메커니즘이 없기 때문에 지속적으로 누적된다. 외부 보정이 없다면 충분히 긴 시간 동안 운용한 후에는 결국 허용할 수 없는 수준의 자세 오차가 발생한다.

서로 다른 오도메트리 기술(odometry technology)은 서로 다른 드리프트 특성을 가진다. 휠 오도메트리(wheel odometry)는 평탄하고 마찰력이 높은 노면에서는 높은 성능을 제공할 수 있지만 미끄러짐, 스키딩(skidding), 연석 통과(curb crossing), 급격한 회전 상황에서는 성능이 저하된다. 관성 오도메트리(inertial odometry)는 각속도와 가속도를 직접 측정하지만 센서 바이어스와 적분 오차의 영향을 받는다. 시각 및 라이다 오도메트리는 환경 관측을 이용하여 운동을 추정함으로써 휠 접촉에 대한 의존성을 낮추지만 시각적 텍스처 또는 기하학적 구조가 부족하면 성능이 저하될 수 있다.

관성 추측항법(inertial dead reckoning)은 오차 누적의 중요성을 특히 명확하게 보여준다. 자이로스코프(gyroscope) 측정값은 자세를 추정하기 위해 적분되고, 가속도계(accelerometer) 측정값은 한 번 적분하여 속도를 계산하고 다시 적분하여 위치를 추정할 수 있다. 따라서 작은 일정 바이어스(constant bias)도 적분 과정에서 증가하여 빠르게 추정 상태를 지배할 수 있다. 유용한 관성항법(inertial navigation)을 구현하려면 정밀한 보정, 바이어스 추정(bias estimation), 중력 보상(gravity compensation), 온도 모델링(temperature modeling), 다른 센서와의 융합이 필수적이다.

휠 측정값과 관성 측정값은 서로 보완적인 오차 특성을 가지기 때문에 자주 결합된다. 정상적인 접지 조건에서 휠 엔코더는 이동 거리에 대한 안정적인 추정값을 제공하고, 관성측정장치(IMU)는 빠른 회전 및 동적 운동 정보를 제공한다. 센서 융합(sensor fusion)은 단기 잡음을 감소시키고 강건성(robustness)을 향상시킬 수 있지만 융합되는 모든 센서가 상대 운동 정보만 제공한다면 전역 드리프트를 근본적으로 제거할 수 없다. 장기적인 오차를 제한하기 위해서는 결국 절대 기준 또는 지도 기준 관측(map-referenced observation)이 필요하다.

시각 오도메트리(visual odometry)는 연속적인 영상 프레임 사이에서 영상 특징(image feature), 광학 흐름(optical flow), 또는 학습된 표현(learned representation)을 추적하여 카메라의 운동을 추정한다. 스테레오 또는 깊이 카메라(stereo or depth camera)는 미터 단위 스케일(metric scale)을 직접 제공할 수 있지만 단안 시스템(monocular system)은 스케일을 결정하기 위해 추가적인 제약조건이 필요할 수 있다. 시각 정보가 풍부한 환경에서는 정확한 상대 궤적을 생성할 수 있지만 조명 변화, 모션 블러(motion blur), 텍스처가 없는 표면, 동적 객체(dynamic object), 반복적인 시각 패턴은 드리프트 또는 추적 실패를 발생시킬 수 있다.

라이다 오도메트리(LiDAR odometry)는 연속적인 스캔 또는 로컬 서브맵(local submap) 사이의 기하학적 관측을 정합하여 상대 운동을 추정한다. 환경에 충분한 기하학적 구조가 존재하고 초기 정렬(initial alignment)이 적절하면 스캔 정합(scan matching)을 통해 정확한 병진과 회전을 계산할 수 있다. 그러나 긴 복도, 개방된 공간, 반복 구조, 식생, 먼지 또는 많은 이동 객체가 존재하는 환경에서는 성능이 저하될 수 있다. 시각 오도메트리와 마찬가지로 라이다 오도메트리도 지속적인 지도 제약조건(persistent map constraint)에 연결되지 않는다면 본질적으로 상대적인 위치추정 방법이다.

드리프트는 반드시 시간이나 거리에 비례하여 일정하게 증가하는 것은 아니다. 예측 가능한 노면에서는 수백 미터를 이동하면서도 오차가 거의 발생하지 않을 수 있지만, 단 한 번의 심각한 슬립(slip)으로 큰 오차가 누적될 수도 있다. 따라서 오차 증가는 궤적 형상(trajectory geometry), 차량 동역학(vehicle dynamics), 지형(terrain), 센서 품질(sensor quality), 보정 상태, 환경 관측 가능성(environmental observability), 운행 속도에 따라 달라진다. 평균적인 백분율 오차만으로 오도메트리를 평가하면 발생 빈도는 낮지만 실제 운용에서 중요한 고장 형태를 놓칠 수 있다.

불확실성 전파(uncertainty propagation)는 누적 오차를 보다 유용하게 표현할 수 있는 방법이다. 각각의 증분 운동 추정값에는 불확실성이 존재하며, 로봇 자세가 갱신될 때 이러한 불확실성도 함께 전파된다. 공분산(covariance)은 이동 거리에 따라 증가할 수 있으며 차량 운동에 따라 방향별로 비대칭적인 형태를 가질 수 있다. 직선 주행에서는 횡방향 불확실성이 헤딩 불확실성의 영향을 크게 받을 수 있고, 회전 운동에서는 병진 및 회전 오차가 더욱 복잡하게 결합될 수 있다.

오도메트리 좌표계(odom frame)는 이러한 특성을 수용하기 위해 특별히 설계된 좌표계이다. 이 좌표계는 갑작스러운 전역 보정 없이 로봇 운동을 적분할 수 있는 국부적으로 연속적인 기준을 제공한다. 오도메트리 좌표계에서 로봇 베이스(robot base)까지의 변환은 부드럽게 변화하며, 전역 위치추정이 누적된 드리프트를 보정할 때는 별도의 맵-오도메트리(map-to-odom) 변환을 조정할 수 있다. 이러한 아키텍처는 제어기에 필요한 연속성을 유지하면서 전체 내비게이션 시스템의 전역 일관성(global consistency)을 확보한다.

드리프트 보정(drift correction)을 위해서는 누적된 상대 궤적과 독립적인 정보를 제공하는 관측값이 필요하다. GNSS는 야외에서 전역 위치를 제공할 수 있고, 지도 정합(map matching)은 라이다 또는 카메라 관측값을 기존에 구축된 환경 모델과 정렬할 수 있다. 인공 표식(fiducial marker), 초광대역 앵커(UWB anchor), 측량된 랜드마크(surveyed landmark), 자기 기준(magnetic reference), 루프 폐쇄(loop closure)도 추가적인 절대 또는 준절대 제약조건을 제공할 수 있다. 이러한 측정값은 오도메트리를 대체하는 것이 아니라 누적된 오차를 주기적으로 제한하는 역할을 한다.

동시적 위치추정 및 지도작성(SLAM)의 루프 폐쇄(loop closure)는 특히 중요한 사례이다. 로봇이 이전에 관측했던 위치로 다시 돌아왔을 때 동일한 장소임을 인식하면 장시간의 추측항법으로 분리되어 있던 상태 사이에 새로운 제약조건을 추가할 수 있다. 이후 최적화(optimization)를 통해 누적된 오차를 전체 추정 궤적에 재분배하고 전역 일관성을 복원할 수 있다. 이 과정에서 맵 좌표계의 궤적은 상당히 수정될 수 있지만 국부 오도메트리 궤적은 연속성을 유지한다.

슬립 검출(slip detection)은 큰 오차가 누적되기 전에 추측항법의 신뢰성을 향상시킬 수 있다. 위치추정 시스템은 휠 기반 속도를 관성 가속도(inertial acceleration), 시각 운동(visual motion), 라이다 운동(LiDAR motion), 모터 전류(motor current), 차량 동역학과 비교하여 서로 일치하지 않는 거동을 식별할 수 있다. 슬립이 감지되면 엔코더 회전을 실제 지면 이동으로 그대로 간주하는 대신 휠 오도메트리에 대한 신뢰도를 낮출 수 있다. 이러한 적응형 측정 가중치(adaptive measurement weighting)는 다양한 노면을 주행하는 야외 자율이동로봇(Outdoor AMR)에서 특히 중요하다.

보정(calibration)은 일회성 실험실 작업이 아니라 지속적인 운용 과정으로 다루어야 한다. 타이어 마모(tire wear), 공기압(inflation pressure), 페이로드, 서스펜션 변형(suspension deflection), 기계 수리, 온도, 구동계 노화(drivetrain aging)는 실질적인 오도메트리 파라미터를 변화시킬 수 있다. 주기적인 보정과 자동 파라미터 추정(automated parameter estimation)을 적용하면 차량 수명주기 동안 정확도를 유지할 수 있다. 독립적인 운동 정보 사이의 잔차(residual)를 모니터링하면 심각한 위치추정 문제가 발생하기 전에 점진적인 보정 성능 저하를 감지할 수 있다.

추측항법의 평가는 서로 다른 오차 메커니즘을 드러낼 수 있는 궤적을 사용해야 한다. 직선 경로는 스케일 및 헤딩 바이어스를 확인할 수 있고, 반복 회전은 각도 보정 오차를 드러내며, 폐쇄 경로(closed loop)는 누적된 위치 및 자세 불일치를 확인할 수 있다. 시험에는 다양한 속도, 페이로드, 노면, 경사, 회전, 슬립 조건이 포함되어야 한다. 상대 자세 오차(relative pose error), 이동 거리당 드리프트(drift per traveled distance), 헤딩 드리프트(heading drift), 루프 폐쇄 오차(loop-closure error)와 같은 지표는 하나의 최종 위치 오차보다 더 많은 정보를 제공한다.

야외 자율이동로봇에서는 RTK-GNSS 또는 지도 기반 위치추정(map localization)을 사용할 수 있는 경우에도 추측항법이 필수적이다. 건물, 나무, 교량, 터널, 전자기 간섭(electromagnetic interference), 다중경로(multipath), 의도적인 GNSS 교란(GNSS disruption)은 전역 기준의 사용을 일시적으로 차단하거나 성능을 저하시킬 수 있다. 고주기의 휠, 관성, 시각 또는 라이다 오도메트리는 이러한 구간에서도 차량이 국부 운동을 지속적으로 추정할 수 있도록 한다. 이 국부 추정의 품질은 전역 보정이 필요해지기 전까지 로봇이 얼마나 오랫동안 안전하게 운용될 수 있는지를 결정한다.

따라서 강건한 자율주행 아키텍처(robust autonomy architecture)는 추측항법을 영구적으로 정확한 전역 위치추정 소스가 아니라 고주기의 국부 운동 백본(high-rate local motion backbone)으로 취급한다. 오도메트리는 연속성(continuity), 응답성(responsiveness), 단기 정밀도(short-term precision)를 제공하고, 전역 측정(global measurement)은 장기적인 일관성을 제공한다. 센서 융합은 이러한 상호 보완적인 역할을 연결하고 불확실성을 추적함으로써 내비게이션 시스템이 누적된 드리프트가 허용 가능한지 또는 보정 관측(corrective observation)이 필요한지를 판단할 수 있도록 한다.

궁극적으로 추측항법은 로봇 위치추정의 근본적인 원리를 보여준다. 정확한 증분 운동(incremental motion)이 반드시 정확한 전역 위치(global position)를 보장하는 것은 아니다. 모든 상대 추정값에는 일정한 오차가 포함되며, 재귀적인 적분은 이러한 작은 오차를 누적된 드리프트로 변화시킨다. 따라서 신뢰성 높은 자율 내비게이션은 추측항법 오차를 완전히 제거하는 것이 아니라 그 불확실성을 모델링하고, 비정상적인 성능 저하를 감지하며, 상호 보완적인 운동 센서를 결합하고, 신뢰할 수 있는 외부 기준에 추정값을 주기적으로 고정함으로써 구현된다.

## 01.04. Bayesian Filtering Theory Kalman Particle Basis

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

베이지안 필터링(Bayesian filtering)은 불확실한 운동(uncertain motion)과 잡음이 포함된 관측(noisy observation)으로부터 동적 시스템(dynamic system)의 숨겨진 상태(hidden state)를 추정하기 위한 수학적 프레임워크이다. 로봇 위치추정(robot localization)에서는 실제 자세(true pose)를 일반적으로 직접 관측할 수 없으므로 로봇은 가능한 상태에 대한 확률분포(probability distribution)를 유지한다. 로봇이 이동하고 새로운 측정값을 획득할 때마다 이 분포를 재귀적으로 예측하고 보정함으로써 각각의 자세 추정값을 정확한 값으로 취급하지 않고 불확실성(uncertainty)을 명시적으로 표현할 수 있다.

베이지안 필터링에서 핵심적인 물리량은 일반적으로 bel(xₜ)로 표현되는 믿음(belief)이며, 이는 시간 t까지 이용 가능한 모든 제어 입력(control)과 관측값(observation)을 고려했을 때 로봇 상태 xₜ가 가질 수 있는 확률분포를 의미한다. 베이지안 추정(Bayesian estimation)은 단순히 로봇이 어디에 있는지를 묻는 것이 아니라 각각의 가능한 상태가 얼마나 높은 확률을 가지는지를 판단한다. 집중된 믿음(concentrated belief)은 높은 신뢰도를 의미하며, 넓거나 다봉 형태의 믿음(multimodal belief)은 높은 불확실성 또는 서로 경쟁하는 여러 위치추정 가설(localization hypothesis)이 존재함을 의미한다.

베이지안 필터(Bayesian filter)는 매번 전체 측정 이력을 처음부터 다시 처리할 필요가 없기 때문에 재귀적으로 동작한다. 마르코프 가정(Markov assumption)에 따르면 현재 제어 입력을 알고 있을 때 현재 상태는 다음 상태를 예측하는 데 필요한 정보를 포함한다고 가정할 수 있다. 마찬가지로 현재 관측값은 주로 현재 상태에 의존한다고 모델링한다. 이러한 가정을 통해 자율 로봇에서 지속적인 확률적 상태 추정(probabilistic state estimation)을 계산적으로 실용적인 형태로 구현할 수 있다.

재귀 과정의 첫 번째 단계는 예측(prediction)이다. 이전의 사후 믿음(posterior belief)과 제어 입력 uₜ가 주어지면 운동 모델(motion model) p(xₜ\|xₜ₋₁,uₜ)은 확률이 새로운 상태 공간으로 어떻게 전파되는지를 예측한다. 로봇의 운동에는 불확실성이 존재하므로 예측된 분포는 일반적으로 더 넓어진다. 휠 슬립(wheel slip), 액추에이터 오차(actuator error), 불완전한 동역학(imperfect dynamics), 프로세스 외란(process disturbance)은 명령된 운동이 정확하고 결정론적인 변위를 발생시킨다고 가정하는 대신 프로세스 불확실성(process uncertainty)으로 표현된다.

수학적으로 예측 과정은 가능한 모든 이전 상태에 대해 적분을 수행한다. 각각의 이전 상태는 상태 전이 모델(state transition model)에 따라 가능한 새로운 상태에 일정한 확률을 제공한다. 그 결과는 최신 센서 관측값이 반영되기 전의 사전 믿음(prior belief)이 된다. 이 연산은 이전 상태를 정확하게 알고 있더라도 불확실한 운동을 수행하면서 정보성이 높은 외부 측정값을 획득하지 못하면 로봇의 상태에 대한 신뢰도가 점차 감소한다는 중요한 위치추정 원리를 표현한다.

두 번째 단계는 측정 보정(measurement correction), 즉 베이지안 갱신(Bayesian update)이다. 센서 관측값 zₜ는 우도 모델(likelihood model) p(zₜ\|xₜ)을 사용하여 평가되며, 이 모델은 로봇이 특정 후보 상태에 존재한다고 가정했을 때 해당 측정값이 발생할 가능성을 나타낸다. 실제 측정값을 잘 설명하는 후보 상태는 더 높은 확률을 부여받고 일치하지 않는 상태는 억제된다. 정규화(normalization)를 수행한 이후 얻어진 사후 분포(posterior)는 다음 예측 과정의 시작점으로 사용되는 새로운 믿음이 된다.

베이즈 정리(Bayes' rule)는 이러한 보정 과정의 개념적 기반을 제공한다. 사후 확률(posterior probability)은 예측된 사전 확률(prior probability)과 측정 우도(measurement likelihood)의 곱에 비례한다. 사전 확률은 새로운 센서 데이터를 관측하기 전에 로봇이 가지고 있던 상태에 대한 믿음을 나타내고, 우도는 새로운 측정값이 가능한 상태에 대해 제공하는 정보를 나타낸다. 두 정보를 결합함으로써 기존에 누적된 지식과 새롭게 획득된 증거를 원칙적으로 균형 있게 반영할 수 있다.

베이지안 필터의 동작은 사용되는 확률 모델(probabilistic model)의 품질에 크게 의존한다. 프로세스 잡음(process noise)을 실제보다 작게 설정하면 추정기가 운동 예측에 지나치게 높은 신뢰도를 가지게 되어 유용한 보정 관측값을 거부할 수 있다. 반대로 측정 잡음(measurement noise)을 과소평가하면 잡음이 포함된 센서 측정값으로 인해 상태 추정값이 불안정하게 변화할 수 있다. 따라서 정확한 불확실성 모델링(uncertainty modeling)은 단순한 수학적 세부사항이 아니라 각각의 정보원이 로봇 상태 추정에 얼마나 큰 영향을 미치는지를 결정하는 핵심 요소이다.

칼만 필터(Kalman filter)는 선형 시스템(linear system)과 가우시안 불확실성(Gaussian uncertainty)을 대상으로 하는 계산 효율적인 베이지안 필터링 구현 방법이다. 전체 확률분포를 직접 저장하는 대신 평균 벡터(mean vector)와 공분산 행렬(covariance matrix)을 사용하여 믿음을 표현한다. 평균은 추정된 상태를 나타내고 공분산은 상태 변수의 불확실성과 상관관계(correlation)를 나타낸다. 이러한 가정이 만족될 경우 칼만 필터는 사용 가능한 정보로부터 최적의 최소분산 추정값(minimum-variance estimate)을 생성한다.

칼만 예측(Kalman prediction) 과정에서는 시스템 모델(system model)을 통해 상태 추정값을 전파하고 프로세스 불확실성에 따라 공분산을 증가시킨다. 보정 과정에서는 예측된 측정값과 실제 측정값을 비교하여 혁신(innovation) 또는 잔차(residual)를 계산한다. 칼만 이득(Kalman gain)은 이 잔차가 상태 추정값을 얼마나 강하게 수정할지를 결정한다. 칼만 이득은 수동으로 선택한 고정 가중치가 아니라 예측과 측정의 상대적인 불확실성을 기반으로 결정된다.

공분산 갱신(covariance update)은 상태 갱신만큼 중요하다. 특정 상태 변수를 강하게 관측할 수 있는 측정값은 해당 변수의 불확실성을 감소시킬 수 있으며, 상태 변수 사이의 상관관계를 통해 다른 변수의 불확실성까지 감소시킬 수 있다. 반대로 정보가 부족하거나 잡음이 큰 관측값은 불확실성을 상대적으로 적게 감소시킨다. 이러한 메커니즘을 통해 추정기는 최적 상태 추정값뿐만 아니라 해당 추정값을 얼마나 신뢰할 수 있는지를 정량적으로 표현할 수 있다.

실제 로봇 시스템은 완벽한 선형 시스템인 경우가 드물기 때문에 확장 칼만 필터(Extended Kalman Filter, EKF)와 무향 칼만 필터(Unscented Kalman Filter, UKF)와 같은 확장 방법이 사용된다. 확장 칼만 필터는 야코비안 행렬(Jacobian matrix)을 이용하여 현재 추정값 주변에서 비선형 운동 및 관측 모델을 선형화한다. 무향 칼만 필터는 신중하게 선택된 시그마 포인트(sigma point)를 비선형 함수를 통해 전파한다. 두 방법 모두 가우시안 믿음 표현을 유지하면서 비선형 동역학과 센싱 문제를 처리한다.

확장 칼만 필터는 많은 일반적인 로봇 모델이 어느 정도 비선형적이면서 계산 자원이 제한되어 있기 때문에 이동 로봇 위치추정에서 역사적으로 중요한 역할을 수행해 왔다. 휠 오도메트리(wheel odometry), 관성측정장치(IMU) 측정값, 위성항법시스템(GNSS) 관측값, 랜드마크 측정(landmark measurement)을 하나의 공통 상태 추정기에서 결합할 수 있다. 그러나 강한 비선형성(strong nonlinearity), 부정확한 초기화(poor initialization), 잘못된 야코비안, 심각한 비가우시안 불확실성(non-Gaussian uncertainty)은 추정의 비일관성(inconsistency)이나 발산(divergence)을 발생시킬 수 있다.

파티클 필터(particle filter)는 베이지안 필터링을 근사하는 또 다른 방법을 제공한다. 하나의 가우시안 분포로 믿음을 표현하는 대신 파티클(particle)이라고 하는 가중 샘플(weighted sample)의 집합을 사용하여 확률분포를 근사한다. 각각의 파티클은 가능한 로봇 상태를 나타내며, 파티클의 가중치는 해당 가설이 관측값을 얼마나 잘 설명하는지를 나타낸다. 이러한 표현을 사용하면 하나의 평균과 공분산으로 적절하게 표현하기 어려운 임의의 비선형적이고 다봉 형태의 확률분포를 유지할 수 있다.

파티클 필터 예측(particle-filter prediction)에서는 각각의 파티클을 샘플링된 프로세스 잡음과 함께 운동 모델을 통해 전파한다. 측정 갱신 과정에서는 해당 상태에서 센서 관측값이 발생할 우도에 따라 각각의 파티클에 가중치를 부여한다. 측정값과 일치하는 파티클은 높은 가중치를 받고 가능성이 낮은 가설은 작은 가중치를 받는다. 이렇게 생성된 가중 파티클 집합(weighted particle population)은 로봇 상태의 사후 확률분포를 근사한다.

재표본추출(resampling)은 파티클 필터를 특징짓는 핵심 연산이다. 반복적인 갱신 이후에는 확률이 소수의 높은 가중치를 가진 파티클에 집중되고 대부분의 파티클은 거의 기여하지 않는 상태가 될 수 있다. 재표본추출은 높은 가중치를 가진 파티클을 우선적으로 복제하고 낮은 가중치의 파티클을 제거하여 새로운 파티클 집합을 생성한다. 이를 통해 계산 자원을 가능성이 높은 상태 공간에 집중할 수 있지만 지나치게 자주 수행하면 파티클 다양성(particle diversity)이 감소할 수 있다.

파티클 필터는 전역 위치추정(global localization)과 납치된 로봇 문제(kidnapped-robot problem)에 특히 유용하다. 로봇의 초기 자세를 알 수 없는 경우 서로 멀리 떨어진 여러 위치가 동시에 가능한 상태일 수 있다. 가우시안 추정기(Gaussian estimator)는 이러한 경쟁 가설을 하나의 분포로 자연스럽게 표현하기 어렵지만 파티클은 지도상의 여러 영역에 동시에 분포할 수 있다. 센서 관측값이 누적되면서 일치하지 않는 가설이 제거되고 확률은 올바른 위치 주변으로 집중된다.

몬테카를로 위치추정(Monte Carlo Localization, MCL)은 지도 기반 로봇 위치추정에 파티클 필터를 적용한 대표적인 방법이다. 파티클은 알려진 지도 내부에서 후보 자세(candidate pose)를 표현하고, 운동 모델은 오도메트리에 따라 파티클을 전파하며, 라이다 거리와 같은 센서 관측값을 이용하여 각각의 우도를 계산한다. 적응형 방식(adaptive variant)은 위치추정 불확실성에 따라 파티클 수를 변경하여 모호성이 높은 상황에서는 더 많은 가설을 사용하고 자세가 충분히 제한된 이후에는 더 적은 파티클을 사용할 수 있다.

파티클 필터링은 계산 비용과 성능 사이의 절충(tradeoff)을 발생시킨다. 많은 수의 파티클을 사용하면 복잡한 확률분포를 보다 정확하게 표현할 수 있지만 처리 비용이 증가한다. 특히 고차원 상태 공간(high-dimensional state space)은 적절한 공간 범위를 표현하는 데 필요한 파티클 수가 빠르게 증가할 수 있기 때문에 어려운 문제이다. 파티클 빈곤(particle impoverishment), 부적절한 제안 분포(poor proposal distribution), 부정확한 우도 모델, 부족한 파티클 수는 추정기가 올바른 가설을 잃게 만들 수 있다.

따라서 칼만 필터와 파티클 필터는 모든 상황에서 경쟁하는 하나의 해법이라기보다 서로 다른 불확실성 구조를 처리하기 위한 방법이다. 칼만 필터 계열은 상태 분포가 대략적인 가우시안 형태를 유지하고 좋은 국부 추정값이 존재할 때 계산 효율성이 높다. 파티클 필터는 서로 다른 여러 가설을 동시에 유지해야 하거나 불확실성이 강한 비가우시안 형태를 가질 때 적합하다. 실제 위치추정 시스템에서는 아키텍처의 서로 다른 계층에서 두 방법을 함께 사용할 수도 있다.

예를 들어 파티클 필터는 지도 내에서 로봇의 전역 자세(global pose)를 결정하고, 확장 칼만 필터는 관성측정장치, 휠 오도메트리, GNSS 측정값을 지속적으로 융합하여 고주기의 국부 상태 추정(high-rate local state estimation)을 수행할 수 있다. 전역 위치추정의 신뢰성이 충분히 확보되면 그 결과를 국부 추정기의 보정값으로 사용하거나 맵-오도메트리(map-to-odom) 관계를 갱신하는 데 이용할 수 있다. 이러한 계층적 추정(hierarchical estimation)은 전역적인 모호성과 고주기의 연속적인 운동 추정을 분리한다.

측정 연관(measurement association)은 베이지안 위치추정에서 또 하나의 근본적인 문제이다. 랜드마크 관측값이 상태를 갱신하기 전에 추정기는 어떤 지도 랜드마크가 해당 측정값을 생성했는지를 판단해야 할 수 있다. 잘못된 연관은 높은 신뢰도를 가지지만 실제로는 잘못된 상태 추정값을 생성할 수 있다. 따라서 확률적 게이팅(probabilistic gating), 기하학적 일관성 검사(geometric consistency check), 디스크립터 정합(descriptor matching), 강건 통계(robust statistics), 다중 가설 추론(multiple-hypothesis reasoning)은 실제 베이지안 필터의 성능과 밀접하게 연결된다.

센서 고장(sensor failure)과 이상치(outlier) 역시 표준 필터링 모델의 가정을 어렵게 만든다. GNSS 다중경로(GNSS multipath), 라이다 반사(LiDAR reflection), 시각 정합 오류(visual mismatch), 엔코더 슬립(encoder slip), IMU 포화(IMU saturation)는 예상된 가우시안 잡음 분포를 크게 벗어나는 측정값을 생성할 수 있다. 강건 필터(robust filter)는 이러한 관측값을 제거하거나 가중치를 낮추거나 명시적으로 모델링할 수 있다. 혁신 모니터링(innovation monitoring)과 적응형 공분산 조정(adaptive covariance adjustment)을 사용하면 운용 조건 변화에 따라 센서 신뢰도를 조정할 수 있다.

베이지안 필터링은 각각의 정보원을 확률 모델로 표현할 수 있기 때문에 센서 융합(sensor fusion)을 위한 자연스러운 기반을 제공한다. 고주기 오도메트리는 운동을 예측하고, 관성측정장치는 동역학을 제약하며, GNSS는 전역 위치를 제공하고, 라이다 또는 카메라 관측은 환경 구조에 대한 자세를 제약한다. 필터는 모델링된 불확실성에 따라 이러한 정보원을 결합하여 통합된 상태 추정값(unified state estimate)과 신뢰도 정보를 함께 생성한다.

야외 자율이동로봇(Outdoor AMR)에서는 센싱 품질이 지속적으로 변화하기 때문에 이러한 확률적 해석이 필수적이다. GNSS는 RTK 수준의 정확도에서 다중경로 상태 또는 완전한 음영 상태로 전환될 수 있고, 휠 오도메트리는 자갈이나 진흙에서 성능이 저하될 수 있으며, 시각 또는 라이다 위치추정은 특징이 부족한 환경에서 모호해질 수 있다. 따라서 추정기는 전체 운용 환경에서 각각의 센서가 항상 동일한 신뢰도를 가진다고 가정하는 대신 측정 정보원의 영향력을 상황에 따라 적응적으로 조절해야 한다.

궁극적으로 베이지안 필터링은 위치추정(localization)을 결정론적인 좌표 계산(deterministic coordinate calculation)에서 순차적인 확률적 추론(sequential probabilistic inference) 문제로 전환한다. 예측은 운동에 따라 불확실성이 어떻게 변화하는지를 설명하고, 측정 보정은 새로운 증거를 반영하며, 사후 믿음은 현재 로봇 상태에 대한 지식을 요약한다. 칼만 필터는 효율적인 가우시안 추정(Gaussian estimation)을 제공하고, 파티클 필터는 복잡하고 다봉 형태의 불확실성을 표현함으로써 현대 로봇 위치추정과 센서 융합의 핵심적인 이론적 기반을 형성한다.

## 01.05. Kalman Filter Extended and Unscented Variants [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

칼만 필터(Kalman Filter)는 불확실한 시스템 예측(system prediction)과 잡음이 포함된 센서 측정(sensor measurement)을 결합하는 재귀적 상태 추정(recursive state estimation) 방법이다. 어느 하나의 정보원을 완벽하게 신뢰하는 대신 추정 상태를 평균(mean)과 공분산(covariance)으로 표현하고 예측과 관측 사이의 균형을 지속적으로 조정한다. 이러한 구조로 인해 칼만 필터링(Kalman filtering)은 로봇 위치추정(robot localization), 내비게이션(navigation), 추적(tracking), 센서 융합(sensor fusion)을 비롯한 다양한 동적 상태 추정 문제의 핵심 기반이 된다.

고전적 칼만 필터(classical Kalman filter)는 시스템 동역학(system dynamics)과 측정 관계(measurement relationship)를 선형 모델(linear model)로 표현할 수 있으며 관련된 불확실성이 가우시안 분포(Gaussian distribution)를 따른다고 가정한다. 상태 전이 모델(state transition model)은 상태가 어떻게 변화하는지를 예측하고, 관측 모델(observation model)은 해당 상태에서 센서가 무엇을 측정해야 하는지를 예측한다. 프로세스 잡음(process noise)은 동역학의 불확실성을 나타내고, 측정 잡음(measurement noise)은 센싱 과정에서 발생하는 불확실성을 나타낸다.

일반적인 상태 벡터(state vector)에는 위치(position), 자세(orientation), 속도(velocity), 가속도 관련 항(acceleration-related term), 센서 바이어스(sensor bias)가 포함될 수 있다. 시간 k에서 필터는 상태 추정값 x̂ₖ와 공분산 Pₖ를 유지한다. 공분산은 단순한 오차 크기가 아니라 각각의 상태 변수에 대한 불확실성과 변수 사이의 상관관계(correlation)를 표현한다. 이를 통해 한 상태 변수에 대해 획득한 정보가 통계적으로 연결된 다른 상태 변수에도 영향을 줄 수 있다.

칼만 필터링 주기(Kalman filtering cycle)는 예측(prediction)으로 시작한다. 이전 상태 추정값은 사용 가능한 제어 정보(control information)와 시스템 모델을 통해 전파되어 다음 관측값이 처리되기 전의 예측 상태(predicted state)를 생성한다. 동시에 공분산도 전파되며 프로세스 잡음 공분산(process-noise covariance) Q에 따라 증가한다. 따라서 수학적으로 계산된 궤적이 부드럽게 보이더라도 불확실한 운동은 자연스럽게 예측 상태에 대한 신뢰도를 감소시킨다.

측정 갱신(measurement update)은 예측된 상태를 이용하여 센서가 무엇을 관측해야 하는지를 계산하는 과정에서 시작한다. 실제 측정값과 예측 측정값 사이의 차이를 혁신(innovation) 또는 잔차(residual)라고 한다. 혁신 공분산(innovation covariance)은 상태 불확실성과 센서 불확실성을 모두 고려했을 때 이러한 차이가 어느 정도 예상 가능한지를 나타낸다. 따라서 혁신 분석(innovation analysis)은 상태 보정뿐만 아니라 일관되지 않은 측정값, 모델 오류, 센서 성능 저하를 검출하는 데도 유용하다.

칼만 이득(Kalman gain)은 혁신이 상태 추정값을 얼마나 강하게 변경할지를 결정한다. 예측 상태의 불확실성이 크고 측정값의 정확도가 높으면 관측값에 더 큰 영향력을 부여한다. 반대로 센서 측정값의 잡음이 크고 예측값의 신뢰도가 높으면 보정량은 작아진다. 이러한 적응형 가중(adaptive weighting)은 융합 강도가 고정된 수동 가중치가 아니라 불확실성 모델로부터 수학적으로 결정된다는 점에서 칼만 필터링의 가장 중요한 특성 중 하나이다.

보정 이후에는 측정값으로부터 획득한 정보를 반영하기 위해 공분산도 갱신된다. 정보성이 높은 관측값(highly informative observation)은 불확실성을 크게 감소시킬 수 있지만, 정보가 부족한 측정값은 작은 수준의 개선만 제공한다. 따라서 예측과 보정 과정은 운동에 의해 불확실성이 증가하고 유용한 관측에 의해 다시 감소하는 반복적인 패턴을 형성한다. 갱신된 사후 상태(posterior state)와 공분산은 다음 주기의 시작점이 된다.

그러나 대부분의 실제 로봇 위치추정 문제는 비선형(nonlinear)이다. 차량 운동에는 헤딩(heading)과 변위(displacement) 사이의 삼각함수 관계가 포함되고, 관성측정장치(IMU) 모델에는 비선형 자세 동역학(nonlinear orientation dynamics)이 포함되며, 센서는 거리(range), 방위각(bearing), 영상 투영 좌표(projected image coordinate)를 관측하는 경우가 많다. 이러한 관계는 고전적인 선형 칼만 필터의 가정을 만족하지 않으므로 확장 칼만 필터(Extended Kalman Filter)와 무향 칼만 필터(Unscented Kalman Filter)와 같은 비선형 변형이 필요하다.

확장 칼만 필터(Extended Kalman Filter, EKF)는 기본적인 예측-보정(prediction-and-correction) 구조를 유지하면서 비선형 함수에 적용할 수 있도록 확장한 방법이다. 상태는 비선형 운동 모델(nonlinear motion model)을 통해 전파되고, 측정값은 비선형 관측 모델(nonlinear observation model)을 통해 예측된다. 공분산을 전파하고 갱신값을 계산하기 위해 EKF는 현재 추정 상태 주변에서 일차 선형화(first-order linearization)를 사용하여 이러한 함수를 국부적으로 근사한다.

이러한 선형화(linearization)는 상태 및 필요한 경우 잡음 변수에 대한 비선형 모델의 편미분(partial derivative)을 포함하는 야코비안 행렬(Jacobian matrix)을 사용하여 수행된다. 야코비안은 현재 추정값 주변의 작은 변화가 비선형 함수를 통해 어떻게 전파되는지를 나타낸다. 따라서 EKF는 비선형 로봇 운동과 센서 모델을 처리하면서도 칼만 필터의 수학적 구조 대부분을 그대로 활용할 수 있다.

EKF는 계산 효율성과 실제적인 정확도 사이에서 우수한 균형을 제공하기 때문에 널리 사용된다. 로봇 시스템에서는 휠 오도메트리(wheel odometry), IMU 데이터, 위성항법시스템(GNSS) 위치, 자기계(magnetometer) 측정값, 시각 관측(visual observation), 라이다 기반 자세 정보(LiDAR-derived pose information)를 융합하는 데 사용할 수 있다. 특히 추정값이 실제 상태에 가깝게 유지되고 공분산으로 표현되는 불확실성 영역에서 비선형성이 심하지 않은 경우 효과적이다.

EKF가 사용하는 근사 방식은 중요한 한계도 발생시킨다. 불확실성이 큰 상황에서는 일차 선형 모델(first-order linear model)이 강한 비선형 함수를 제대로 표현하지 못할 수 있다. 초기 추정값이 부정확하거나 공분산이 과소평가되거나 측정값에 의해 큰 보정이 발생하는 경우 잘못된 상태 주변에서의 선형화가 일관되지 않은 갱신을 생성할 수 있다. 심각한 경우 필터가 지나치게 높은 신뢰도를 가지거나 실제 궤적으로부터 발산(divergence)할 수 있다.

자세 추정(orientation estimation)은 회전이 일반적인 유클리드 벡터(Euclidean vector)와 동일하게 동작하지 않기 때문에 특히 주의가 필요하다. 오일러 각(Euler angle)을 직접 필터링하면 특이점(singularity)과 불연속성(discontinuity)이 발생할 수 있으며, 쿼터니언(quaternion)은 단위 노름 제약(unit-norm constraint)을 유지해야 한다. 따라서 현대적인 관성 위치추정 시스템에서는 명목 자세(nominal pose)를 별도로 전파하고 필터가 이를 보정하는 작은 국부 오차(local error)를 추정하는 오차 상태 방식(error-state formulation)을 자주 사용한다.

오차 상태 확장 칼만 필터(Error-State Extended Kalman Filter, ESKF)는 관성항법(inertial navigation)에서 특히 널리 사용된다. IMU 측정값은 위치, 속도, 자세, 센서 바이어스를 높은 주기로 전파하고 GNSS, 라이다, 카메라 또는 다른 관측값이 누적된 오차를 주기적으로 보정한다. 자세 오차를 국부적으로 표현하면 수치적 특성이 개선되고, 추정기는 간결한 불확실성 표현을 유지하면서 고주기의 비선형 관성 동역학을 처리할 수 있다.

무향 칼만 필터(Unscented Kalman Filter, UKF)는 야코비안 행렬을 명시적으로 계산하지 않고 비선형 상태 추정 문제를 처리한다. 비선형 함수 자체를 선형화하는 대신 시그마 포인트(sigma point)라고 하는 신중하게 선택된 결정론적 샘플(deterministic sample)을 사용하여 확률분포를 근사한다. 이러한 포인트들은 현재 평균 주변에 공분산에 따라 배치되고 원래의 비선형 모델을 통해 직접 전파된다.

시그마 포인트가 비선형 변환을 통과한 후에는 변환된 위치와 가중치를 사용하여 예측 평균과 공분산을 다시 계산한다. 이러한 과정을 무향 변환(unscented transform)이라고 한다. 핵심적인 개념은 비선형 함수를 국부적인 선형 근사로 표현하는 것보다 확률분포가 비선형 함수를 통과하면서 어떻게 변화하는지를 직접 근사하는 것이 더 정확할 수 있다는 것이다.

UKF 예측 과정에서는 현재 상태 분포를 나타내는 시그마 포인트를 비선형 운동 모델을 통해 전파한다. 변환된 시그마 포인트 분포로부터 예측 상태와 공분산을 계산한다. 측정 갱신 과정에서는 시그마 포인트를 비선형 관측 모델을 통해 변환하여 예측 측정값, 측정 공분산(measurement covariance), 그리고 칼만 방식의 보정에 필요한 교차 공분산(cross-covariance)을 계산한다.

UKF는 비선형성이 상당하고 상태 분포가 가우시안 분포로 합리적으로 표현될 수 있는 경우 EKF보다 높은 정확도를 제공할 수 있다. 또한 해석적인 야코비안(analytical Jacobian)을 유도해야 하는 구현 부담과 그 과정에서 발생할 수 있는 오류를 피할 수 있다. 이는 정확한 미분식을 유지하기 어려운 복잡한 로봇 모델이나 개발 과정에서 시스템 방정식이 자주 변경되는 경우 유용하다.

그러나 이러한 장점이 UKF가 모든 상황에서 EKF보다 우수하다는 것을 의미하지는 않는다. 여러 시그마 포인트를 전파하려면 하나의 명목 상태만 전파하는 것보다 더 많은 모델 계산이 필요하므로 상태 차원이 증가할수록 일반적으로 계산 비용도 증가한다. 시그마 포인트 파라미터도 적절하게 선택해야 하며, UKF 역시 기본적으로 평균과 공분산을 통해 믿음(belief)을 표현한다. 따라서 강한 다봉 불확실성(multimodal uncertainty)은 자연스럽게 표현하기 어렵다.

KF, EKF, UKF의 선택은 단순한 알고리즘 성능 순위가 아니라 상태 추정 문제의 구조에 따라 결정되어야 한다. 고전적 칼만 필터(KF)는 모델이 선형이거나 실제적으로 선형으로 취급할 수 있는 경우 적합하다. EKF는 비선형 모델이 충분히 이해되어 있고 계산 자원이 제한된 임베디드 로봇 시스템(embedded robotic system)에 자주 사용된다. UKF는 비선형 효과가 더 강하고 추가적인 계산 비용을 허용할 수 있는 경우 유용한 선택이 된다.

프로세스 잡음 공분산(process-noise covariance) Q와 측정 잡음 공분산(measurement-noise covariance) R은 모든 칼만 계열 추정기에서 매우 중요하다. Q는 상태 전이 모델이 표현하지 못하는 불확실성을 나타내고, R은 센서 관측값의 불확실성을 나타낸다. 방정식이 정확하게 구현되어 있더라도 이러한 값이 잘못 설정되면 추정 성능이 크게 저하될 수 있다. 지나치게 작은 공분산은 과도한 신뢰(overconfidence)를 발생시키고, 지나치게 큰 값은 추정기가 불필요하게 느리거나 측정 변화에 둔감하게 만들 수 있다.

필터 일관성(filter consistency)은 필터가 보고하는 불확실성이 실제 추정 오차와 통계적으로 일치해야 한다는 것을 의미한다. 일반적인 운용 조건에서 필터 출력이 부드럽고 정확해 보이더라도 공분산이 비현실적으로 작다면 필터는 비일관적일 수 있다. 혁신 통계(innovation statistics), 정규화 잔차 검정(normalized residual test), 공분산 분석(covariance analysis), 몬테카를로 시뮬레이션(Monte Carlo simulation), 고품질 기준값(ground truth)과의 비교를 통해 불확실성 추정이 신뢰할 수 있는지를 평가할 수 있다.

강건한 센서 융합(robust sensor fusion)을 위해서는 가정된 잡음 모델을 위반하는 측정값도 처리해야 한다. GNSS 다중경로(GNSS multipath), 휠 슬립(wheel slip), 시각 정합 오류(visual mismatch), 라이다 정합 실패(LiDAR registration failure), 자기장 간섭(magnetic interference)은 큰 이상치(outlier)를 발생시킬 수 있다. 혁신 게이팅(innovation gating)은 통계적으로 가능성이 낮은 잔차를 가진 측정값을 거부할 수 있으며, 적응형 공분산(adaptive covariance) 기법은 성능이 저하된 센서의 영향력을 감소시킬 수 있다. 이러한 메커니즘이 없으면 하나의 잘못된 측정값이 큰 상태 추정 오차를 발생시킬 수 있다.

비동기 센싱(asynchronous sensing)은 또 다른 실질적인 요구사항을 발생시킨다. IMU는 수백 헤르츠(hertz)로 동작할 수 있고, 휠 엔코더는 이보다 낮은 주파수로 동작하며, 카메라와 라이다는 수십 헤르츠, GNSS는 또 다른 주기로 동작할 수 있다. 필터는 각각의 관측값을 적절한 상태 시점과 연결하고 지연(latency)을 고려해야 하며, 측정 이벤트 사이에서 상태를 전파해야 하는 경우도 많다. 따라서 정확한 타임스탬프(timestamp)와 시간 동기화(time synchronization)는 고품질 칼만 기반 센서 융합과 분리할 수 없는 요소이다.

야외 자율이동로봇(Outdoor AMR)에서는 일반적으로 고주기의 IMU와 휠 오도메트리를 이용하여 연속적인 예측을 수행하고, GNSS, 라이다 위치추정(LiDAR localization), 시각 위치추정(visual localization)을 이용하여 상대적으로 낮은 주기의 보정을 수행하는 아키텍처를 사용할 수 있다. GNSS의 신뢰성이 저하되면 측정 공분산을 증가시키거나 측정값을 거부하여 국부 센서가 일시적으로 더 큰 영향력을 가지도록 할 수 있다. 신뢰할 수 있는 전역 관측값이 다시 확보되면 누적된 드리프트를 제한하고 전역 일관성(global consistency)을 복원할 수 있다.

칼만 계열 필터(Kalman-family filter)는 궁극적으로 단순한 평균 계산 알고리즘이 아니라 불확실성 관리 시스템(uncertainty-management system)으로 이해해야 한다. 이들의 강점은 상태와 불확실성을 함께 전파하고, 예측 정보와 관측 정보를 비교하며, 신뢰도에 따라 보정 강도를 조절하는 데 있다. KF는 선형 시스템의 기본 구조를 제공하고, EKF는 국부 선형화(local linearization)를 통해 이를 확장하며, UKF는 시그마 포인트를 전파하여 비선형 변환을 보다 직접적으로 처리한다.

이러한 변형들은 현대 로봇 상태 추정(modern robotic state estimation)의 핵심적인 기반을 형성한다. 적절한 필터를 선택하려면 시스템 동역학, 센서 특성, 비선형성, 계산 자원, 예상되는 고장 조건(failure condition)을 이해해야 한다. 모델, 보정(calibration), 시간 동기화, 공분산 튜닝(covariance tuning), 이상치 처리를 세심하게 설계하면 칼만 계열 추정기는 신뢰성 높은 위치추정, 내비게이션, 경로 계획(planning), 제어에 필요한 연속적이고 불확실성을 고려한 상태 정보를 제공할 수 있다.

## 01.06. Particle Filter Monte Carlo Localization MCL [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

파티클 필터링(Particle Filtering)은 하나의 평균(mean)과 공분산(covariance) 대신 가중치가 부여된 샘플(weighted sample)의 집합을 사용하여 불확실성(uncertainty)을 표현하는 확률적 상태 추정(probabilistic state estimation) 방법이다. 각각의 샘플은 파티클(particle)이라고 하며 가능한 하나의 로봇 상태를 나타내고, 그 가중치(weight)는 해당 가설이 사용 가능한 관측값과 얼마나 잘 일치하는지를 표현한다. 이러한 표현을 통해 위치추정 시스템(localization system)은 복잡하고 비선형적이며 다봉 형태(multimodal)의 확률분포를 유지할 수 있다.

로봇 위치추정(robot localization)에서 하나의 파티클은 일반적으로 2차원 환경에서 xₜ = [x, y, θ]와 같은 후보 자세(candidate pose)를 나타낸다. 수백 개 또는 수천 개의 파티클이 서로 다른 로봇 위치와 방향을 동시에 표현할 수 있다. 이들의 공간적 분포(spatial distribution)는 로봇 자세에 대한 믿음(belief)을 근사한다. 밀집된 파티클 군집(cluster)은 높은 확률을 가진 영역을 의미하며, 넓게 분산된 파티클은 로봇의 실제 위치에 대한 불확실성 또는 모호성(ambiguity)을 나타낸다.

파티클 필터는 샘플링(sampling)을 통해 베이지안 필터링(Bayesian filtering)을 구현한다. 이론적인 베이지안 필터는 전체 확률분포를 운동 모델(motion model)과 측정 모델(measurement model)을 통해 전파하지만 실제적인 상태 공간에서는 계산이 어려울 수 있다. 파티클 필터링은 이러한 연속적인 분포를 유한한 샘플 집합(finite population of samples)으로 대체한다. 이후 예측(prediction), 측정 평가(measurement evaluation), 재표본추출(resampling)을 반복하여 로봇이 이동하고 환경을 관측함에 따라 사후 확률분포(posterior distribution)가 어떻게 변화하는지를 재귀적으로 근사한다.

예측 단계(prediction stage)에서는 각각의 파티클을 로봇의 운동 모델에 따라 전파한다. 휠 오도메트리(wheel odometry), 시각 오도메트리(visual odometry), 라이다 오도메트리(LiDAR odometry), 기타 상대 운동 정보(relative-motion information)를 이용하여 기본적인 이동량을 결정할 수 있다. 전파 과정에는 운동 불확실성을 나타내는 무작위 샘플(random sample)이 추가된다. 따라서 처음에는 비슷한 자세에 있던 파티클도 불확실한 운동이 누적됨에 따라 서로 분산되며, 이를 통해 증가하는 위치추정 불확실성을 자연스럽게 표현할 수 있다.

운동 모델은 단순히 임의의 잡음(arbitrary noise)을 추가하는 것이 아니라 실제적인 로봇 거동을 반영해야 한다. 병진 불확실성(translational uncertainty), 회전 불확실성(rotational uncertainty), 휠 슬립(wheel slip), 조향 오차(steering error), 운동에 따라 변화하는 외란(motion-dependent disturbance)은 파티클 분포에 서로 다른 영향을 줄 수 있다. 직선으로 주행하는 로봇과 급회전을 수행하는 로봇은 서로 다른 형태로 불확실성을 누적할 수 있다. 따라서 정확한 운동 잡음 모델링(motion-noise modeling)은 실제 자세가 파티클 집합 내부에서 지속적으로 표현될 수 있는지를 결정하는 중요한 요소이다.

예측 이후 측정 갱신(measurement update)에서는 각각의 파티클이 현재 센서 관측(sensor observation)을 얼마나 잘 설명하는지를 평가한다. 파티클 자세와 알려진 지도(map)가 주어지면 위치추정 시스템은 해당 위치에서 로봇이 무엇을 관측해야 하는지를 예측한다. 예측된 관측값과 실제 센서 데이터를 비교하여 우도(likelihood)를 계산한다. 예측된 측정값이 실제 환경과 잘 일치하는 파티클에는 더 높은 가중치가 부여된다.

라이다(LiDAR)는 거리 측정값(range measurement)을 기하학적 지도(geometric map)와 효율적으로 비교할 수 있기 때문에 파티클 필터 기반 위치추정에서 자주 사용된다. 각각의 후보 자세에 대해 측정된 거리 또는 스캔 종점(scan endpoint)을 점유 격자 지도(occupancy grid)나 거리장(distance field)의 점유 구조와 비교한다. 실제 자세에 가까운 파티클은 일반적으로 벽과 환경의 기하 구조에 일치하는 센서 관측을 생성하는 반면, 잘못된 가설은 더 낮은 우도를 가지게 된다.

측정 모델(measurement model)은 불완전한 센싱과 불완전한 지도를 허용할 수 있어야 한다. 실제 환경에는 사람, 차량, 이동 가능한 장비, 문, 식생(vegetation), 반사체, 그리고 저장된 지도와 달라진 구조물이 존재할 수 있다. 모든 불일치를 불가능한 상황으로 처리하면 소수의 동적 장애물(dynamic obstacle)만으로도 올바른 파티클이 잘못 제거될 수 있다. 따라서 강건한 우도 모델(robust likelihood model)은 정적 지도 가정과 일치하지 않는 측정값의 영향을 제한하여 위치추정 성능을 유지한다.

파티클 가중치를 계산한 이후에는 전체 파티클 집합이 이산 확률분포(discrete probability distribution)를 형성하도록 가중치를 정규화(normalization)한다. 측정값이 올바른 영역을 명확하게 구별할 수 있는 경우 소수의 파티클이 전체 확률의 대부분을 차지할 수 있다. 반대로 서로 유사하게 보이는 환경에서는 여러 가설이 비슷한 가중치를 유지할 수 있다. 따라서 가중치 분포(weight distribution)는 추정된 자세뿐만 아니라 위치추정의 모호성에 대한 정보도 제공한다.

반복적인 가중치 갱신은 대부분의 파티클이 매우 작은 가중치를 가지며 추정에 거의 기여하지 못하는 파티클 퇴화(particle degeneracy)를 발생시킨다. 재표본추출(resampling)은 정규화된 가중치에 따라 새로운 파티클 집합을 생성하여 이러한 문제를 해결한다. 높은 확률을 가진 파티클은 여러 번 선택될 가능성이 높고 낮은 확률의 파티클은 제거된다. 결과적으로 계산 자원을 센서 관측에 의해 지지되는 가능성 높은 가설 주변에 집중시킬 수 있다.

재표본추출은 지나치게 수행하면 파티클 빈곤(particle impoverishment)을 발생시킬 수 있으므로 신중하게 적용해야 한다. 소수 파티클의 복제본이 전체 집합을 지배하면 다양성(diversity)이 감소하고, 지배적인 가설이 잘못된 경우 필터가 복구하지 못할 수 있다. 운동 잡음, 선택적 재표본추출(selective resampling), 개선된 제안 분포(proposal distribution), 무작위 파티클 주입(random particle injection), 적응형 샘플링(adaptive sampling)을 이용하면 가능성 높은 상태에 파티클을 집중시키면서도 충분한 다양성을 유지할 수 있다.

유효 샘플 크기(effective sample size)는 파티클 퇴화 정도를 실용적으로 판단할 수 있는 지표이다. 정규화된 파티클 가중치의 분포로부터 추정하며, 확률이 소수의 파티클에 집중될수록 값이 작아진다. 모든 센서 갱신 이후 재표본추출을 수행하는 대신 유효 샘플 크기가 설정된 임계값(threshold) 이하로 감소했을 때만 재표본추출을 수행할 수 있다. 이를 통해 불필요한 파티클 다양성 손실과 계산량을 줄일 수 있다.

몬테카를로 위치추정(Monte Carlo Localization, MCL)은 알려진 지도 내에서 로봇의 자세를 추정하는 문제에 파티클 필터링을 적용한 대표적인 방법이다. 파티클은 후보 자세를 나타내고, 운동 모델은 로봇의 이동에 따라 파티클을 전파하며, 센서 모델은 지도와의 일치 정도에 따라 우도를 부여한다. 이러한 재귀적인 과정을 통해 누적된 운동 정보와 센서 관측값을 기반으로 로봇 자세의 사후 확률을 근사한다.

MCL의 주요 장점 중 하나는 전역 위치추정(global localization)을 수행할 수 있다는 것이다. 로봇의 초기 자세를 알 수 없는 경우 하나의 가정된 위치 주변에 파티클을 초기화하는 대신 유효한 지도 전체에 파티클을 분포시킬 수 있다. 로봇이 환경을 관측하고 이동함에 따라 측정값과 일치하지 않는 가설은 점차 확률을 잃는다. 결국 파티클 집합은 누적된 관측값을 잘 설명하는 영역 주변으로 수렴(convergence)한다.

이러한 능력은 하나의 근사적인 가우시안 자세 분포(Gaussian pose distribution)를 가정하는 추정기와 MCL을 구별하는 중요한 특성이다. 예를 들어 대칭적인 창고(symmetric warehouse)에서는 여러 통로가 초기에는 거의 동일한 라이다 관측값을 생성할 수 있다. 파티클 필터는 이러한 가설들을 물리적으로 의미가 없는 중간 위치로 평균화하지 않고 각각의 통로에 대한 가설을 유지할 수 있다. 이후 추가적인 이동과 관측을 통해 모호성을 해소하고 올바른 가설을 선택할 수 있다.

MCL은 로봇이 위치추정 시스템에 알려지지 않은 상태에서 갑자기 다른 위치로 이동되는 납치된 로봇 문제(kidnapped-robot problem)도 처리할 수 있다. 기존 자세 주변에만 집중된 필터는 새로운 관측값을 국부적인 보정만으로 설명할 수 없기 때문에 영구적으로 실패할 수 있다. 복구 메커니즘(recovery mechanism)은 지도의 다른 위치에 새로운 파티클을 도입하여 기존 위치추정과 관측값 사이의 불일치가 증가할 때 새로운 가설이 확률을 획득할 수 있도록 한다.

적응형 몬테카를로 위치추정(Adaptive Monte Carlo Localization, AMCL)은 위치추정 조건에 따라 샘플링 동작을 조절하여 계산 효율성을 향상시킨다. 자세가 충분히 제한된 경우에는 비교적 적은 수의 파티클만으로도 충분할 수 있다. 전역적인 불확실성, 위치 복구, 모호한 센싱 상황에서는 경쟁하는 가설을 표현하기 위해 더 많은 파티클을 사용할 수 있다. 적응형 전략을 사용하면 고정된 파티클 수를 유지하는 대신 위치추정 믿음의 복잡도에 따라 계산 자원을 조절할 수 있다.

쿨백-라이블러 발산 샘플링(KLD-sampling)은 파티클 수를 적응적으로 조절하는 방법 중 하나이다. 현재 분포를 지정된 통계적 오차 범위 내에서 근사하기 위해 필요한 샘플 수를 추정한다. 상태 공간의 작은 영역만 차지하는 집중된 분포는 적은 수의 파티클로 표현할 수 있지만 넓거나 다봉 형태의 분포는 더 많은 파티클을 필요로 한다. 이러한 방식은 운용 과정에서 위치추정 불확실성이 크게 변화하는 시스템에 특히 유용하다.

파티클 필터의 성능은 지도 품질(map quality)과 해상도(resolution)에 크게 의존한다. 오래되어 실제 환경과 달라진 지도, 기하학적으로 왜곡된 지도, 정렬이 잘못된 지도, 지나치게 상세한 지도는 올바른 자세에서도 측정 우도를 감소시킬 수 있다. 반대로 지나치게 단순화된 지도는 서로 유사한 위치를 구별하지 못할 수 있다. 따라서 지도 표현(map representation), 센서 해상도, 우도장 파라미터(likelihood-field parameter), 장애물 처리, 환경의 동적 특성을 위치추정 알고리즘과 함께 설계해야 한다.

초기화 전략(initialization strategy) 역시 수렴 성능에 영향을 준다. 신뢰할 수 있는 대략적인 초기 자세가 존재하는 경우 사전 지식의 불확실성을 반영하여 해당 자세 주변에 파티클을 샘플링할 수 있다. 이를 통해 비교적 적은 수의 샘플만으로 빠른 국부 수렴(local convergence)이 가능하다. 초기 자세에 대한 정보가 없는 경우 전역 초기화(global initialization)를 위해 훨씬 넓은 영역에 파티클을 분포시켜야 하므로 계산량이 증가하며, 관측 정보가 충분히 구별될 수 있을 때까지 로봇의 추가적인 이동이 필요할 수 있다.

파티클 수(particle count)는 표현 품질과 계산 비용 사이의 근본적인 절충(tradeoff)을 만든다. 파티클 수가 너무 적으면 실제 상태를 포함하지 못하거나 경쟁하는 가설을 유지하지 못할 수 있고, 지나치게 많으면 운동 전파, 센서 우도 계산, 메모리 사용량, 처리 지연(latency)이 증가한다. 필요한 파티클 수는 지도 크기, 환경 모호성(environmental ambiguity), 센서 정보량, 불확실성, 그리고 시스템이 국부 추적(local tracking) 또는 전역 위치추정을 수행하는지에 따라 달라진다.

센서 우도 계산(sensor likelihood calculation)은 MCL에서 계산 비용의 대부분을 차지하는 경우가 많다. 조밀한 라이다 스캔의 모든 빔(beam)을 각각의 파티클에 대해 평가하면 특히 파티클 수가 많을 때 높은 계산 비용이 발생한다. 실제 시스템에서는 빔을 부분적으로 샘플링하거나, 거리 변환(distance transform)을 미리 계산하거나, 우도장(likelihood field)을 사용하거나, 계산을 병렬화하거나, 하드웨어 가속(hardware acceleration)을 이용할 수 있다. 이러한 효율화 과정에서도 인접하거나 모호한 자세를 안정적으로 구별할 수 있을 만큼 충분한 측정 정보를 유지해야 한다.

MCL은 여러 센서의 정보를 결합할 수 있지만 확률적 가정(probabilistic assumption)을 신중하게 처리해야 한다. 라이다, 깊이 센싱(depth sensing), 시각 랜드마크(visual landmark), 자기장 특징(magnetic feature), 기타 지도 기준 관측(map-referenced observation)은 우도 정보에 기여할 수 있다. 휠 오도메트리와 IMU 정보는 일반적으로 운동 예측을 지원한다. 상호 보완적인 관측 정보를 결합하면 하나의 센싱 방식이 약해지거나 모호해지거나 일시적으로 사용할 수 없는 상황에서도 강건성을 향상시킬 수 있다.

위치추정 신뢰도(localization confidence)는 가장 높은 가중치를 가진 하나의 파티클만으로 판단해서는 안 된다. 좁고 지배적인 하나의 파티클 군집은 일반적으로 비슷한 총확률을 가진 여러 개의 분리된 군집보다 높은 위치추정 신뢰도를 의미한다. 선택된 군집 주변에서 계산된 공분산(covariance), 파티클 분산(particle dispersion), 유효 샘플 크기, 우도 통계(likelihood statistics), 가설 구조(hypothesis structure)는 추가적인 신뢰도 지표를 제공할 수 있다. 이러한 정보는 경로 계획 및 안전 시스템에도 전달할 수 있다.

잘못된 수렴(incorrect convergence)은 가장 중요한 고장 형태 중 하나이다. 반복적인 환경에서는 파티클이 기하학적으로는 타당하지만 실제로는 잘못된 위치 주변에 집중될 수 있다. 다양성이 사라진 이후에는 새로운 관측값이 들어와도 실제 자세를 쉽게 복구하지 못할 수 있다. 복구용 파티클(recovery particle)을 유지하고, 관측 우도를 모니터링하며, 의미론적 또는 전역적으로 구별 가능한 특징(semantic or globally distinctive feature)을 사용하고, 갑작스러운 불일치를 감지하면 지속적인 오위치추정(false localization)의 가능성을 줄일 수 있다.

야외 자율이동로봇(Outdoor AMR)에서는 지도 기반 전역 위치추정이 필요한 경우 파티클 필터링을 GNSS 및 연속적인 오도메트리와 상호 보완적으로 사용할 수 있다. GNSS는 대략적인 사전 위치(coarse prior)를 제공하고, 라이다 또는 시각 관측은 로컬 지도에 대한 자세를 정밀하게 보정할 수 있다. 다중경로 또는 GNSS 음영의 영향을 받는 지역에서는 MCL이 지도 기준 위치추정을 유지할 수 있다. 위성 기반 전역 정보가 다시 신뢰할 수 있는 상태가 되면 탐색 영역을 제한하고 모호한 지도 형상으로부터 위치를 복구하는 데 도움을 줄 수 있다.

따라서 실제적인 아키텍처에서는 연속적인 국부 상태 추정(continuous local state estimation)과 전역 지도 위치추정(global map localization)을 분리할 수 있다. 확장 칼만 필터(EKF) 또는 오차 상태 확장 칼만 필터(ESKF)는 오도메트리 좌표계(odom frame)에서 IMU와 휠 오도메트리를 높은 주기로 융합하고, MCL은 상대적으로 낮은 주기로 지도에 대한 로봇 자세를 추정할 수 있다. 파티클 필터 결과는 맵-오도메트리 변환(map-to-odom transformation)을 갱신하여 부드러운 국부 운동을 유지하면서 누적된 드리프트를 보정하고 전역 지도 일관성(global map consistency)을 유지할 수 있다.

궁극적으로 파티클 필터링은 모든 믿음을 하나의 가우시안 근사(Gaussian approximation)로 강제하는 대신 명시적으로 경쟁하는 여러 가설을 통해 불확실성을 표현하기 때문에 강력한 위치추정 프레임워크를 제공한다. 몬테카를로 위치추정은 이러한 능력을 지도 기반 자세 추정(map-based pose estimation)에 적용하여 국부 추적, 전역 초기화, 모호성 해소(ambiguity resolution), 위치추정 실패 복구를 지원한다. 그 효과는 현실적인 운동 모델, 강건한 센서 우도, 제어된 재표본추출, 충분한 파티클 다양성, 세심한 계산 설계에 의해 결정된다.

## 01.07. Localization Accuracy Metrics ATE RTE Definition

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

위치추정 정확도(localization accuracy)는 시각적으로 합리적으로 보이는 궤적이라도 상당한 위치(position), 자세(orientation), 스케일(scale), 드리프트(drift) 오차를 포함할 수 있기 때문에 정량적으로 평가해야 한다. 평가는 추정된 로봇 궤적(estimated robot trajectory)을 일반적으로 기준값(ground truth)이라고 하는 신뢰할 수 있는 기준 궤적(reference trajectory)과 비교하여 수행한다. 평가 지표(metric)는 이러한 차이를 수치화하여 위치추정 알고리즘, 센서, 파라미터 설정, 운용 조건을 객관적으로 비교할 수 있도록 한다.

위치추정 오차(localization error)를 계산하기 전에 추정 궤적과 기준 궤적은 시간 및 좌표 표현(coordinate representation)에서 서로 대응되어야 한다. 측정값은 서로 다른 주파수 또는 타임스탬프(timestamp)로 생성될 수 있으므로 시간 연관(temporal association) 또는 보간(interpolation)이 필요할 수 있다. 좌표계(coordinate frame) 역시 일관되어야 한다. 그렇지 않으면 위치추정 알고리즘 자체가 아니라 타임스탬프 오프셋(timestamp offset), 축 규칙(axis convention), 원점 불일치(origin mismatch), 잘못된 강체 변환(rigid transformation)이 위치추정 오차처럼 나타날 수 있다.

추정 궤적이 기준값과 서로 다른 좌표계에서 표현되는 경우가 있으므로 궤적 정렬(trajectory alignment)이 필요한 경우가 많다. 강체 변환을 사용하면 평가 전에 두 궤적 사이의 병진(translation)과 회전(rotation)을 정렬할 수 있다. 단안 시각 오도메트리(monocular visual odometry)는 미터 단위 스케일(metric scale)을 본질적으로 직접 관측할 수 없으므로 추가적인 스케일 정렬(scale alignment)이 필요할 수 있다. 정렬 과정은 실제 운용에서 중요한 오차까지 제거할 수 있으므로 평가 프로토콜에서는 어떤 정렬 자유도(alignment freedom)를 허용하는지 명확하게 정의해야 한다.

절대 궤적 오차(Absolute Trajectory Error, ATE)는 공통 전역 좌표계(common global frame)를 기준으로 추정 자세와 기준 자세 사이의 차이를 측정한다. 적절한 궤적 연관과 정렬을 수행한 후 각각의 타임스탬프에서 추정 자세를 대응하는 기준 자세와 비교한다. 따라서 ATE는 전체 추정 궤적이 로봇의 실제 전역 궤적(global trajectory)을 얼마나 정확하게 재현하는지를 나타낸다.

강체 변환(rigid transformation)으로 표현된 자세의 경우 각각의 타임스탬프에서 정렬된 추정 자세와 기준 자세 사이의 오차 변환(error transformation)을 계산할 수 있다. 이 변환의 병진 성분(translational component)은 절대 위치 오차(absolute position error)를 나타내고, 회전 성분(rotational component)은 자세 오차를 정량화하는 데 사용할 수 있다. 이러한 방식은 평면 위치추정뿐만 아니라 3차원 병진과 회전을 포함하는 완전한 6자유도(six-degree-of-freedom, 6-DoF) 궤적에도 자연스럽게 확장된다.

ATE는 흔히 제곱평균제곱근 오차(Root Mean Square Error, RMSE)를 사용하여 요약한다. eᵢ가 샘플 i에서의 병진 오차 크기를 나타낸다면 병진 ATE RMSE는 모든 샘플에 대한 eᵢ²의 평균에 제곱근을 적용하여 계산한다. 제곱 연산은 큰 오차에 더 높은 영향을 부여하므로 RMSE는 간헐적으로 발생하는 심각한 위치추정 실패에 민감하다. 평균(mean), 중앙값(median), 표준편차(standard deviation), 최솟값(minimum), 최댓값(maximum), 백분위수(percentile)도 상호 보완적인 정보를 제공할 수 있다.

낮은 ATE는 선택된 전역 좌표계에서 추정 궤적이 기준 궤적에 가깝게 유지된다는 것을 의미한다. 그러나 하나의 ATE 값만으로는 오차가 어떤 방식으로 발생했는지를 설명할 수 없다. 두 위치추정 시스템이 비슷한 ATE를 가지더라도 하나는 점진적인 드리프트를 경험하고 다른 하나는 대부분의 경로에서 정확하지만 한 번의 큰 위치추정 실패를 경험했을 수 있다. 따라서 시간에 따른 오차(error-versus-time)와 이동 거리에 따른 오차(error-versus-distance)를 함께 분석하는 것이 중요하다.

상대 궤적 오차(Relative Trajectory Error, RTE)는 절대적인 전역 자세가 아니라 지정된 시간 또는 공간 구간에서 상대 운동(relative motion)의 정확도를 평가한다. 두 개의 추정 자세 사이의 상대 변환(relative transformation)을 대응하는 두 기준 자세 사이의 상대 변환과 비교한다. 따라서 RTE는 특정 구간에서 위치추정 시스템이 운동을 얼마나 정확하게 추정하는지를 측정하며, 특히 국부 드리프트(local drift)를 분석하는 데 유용하다.

RTE는 SLAM과 오도메트리 평가에서 널리 사용되는 상대 자세 오차(Relative Pose Error, RPE)와 밀접하게 관련되어 있다. 평가 규칙에 따라 상대 오차는 연속된 자세 사이, 고정된 시간 간격(fixed time interval), 고정된 이동 거리(fixed traveled distance), 또는 여러 구간 길이에 대해 계산할 수 있다. 선택된 구간은 결과의 의미를 근본적으로 변화시키므로 RTE 결과를 제시할 때는 반드시 평가에 사용된 구간 정의(segment definition)를 함께 명시해야 한다.

짧은 구간의 RTE는 국부 운동 추정(local motion estimation)의 품질을 강조한다. 전역 보정(global correction)에 의해 ATE가 비교적 작게 유지되는 경우에도 엔코더 잡음(encoder noise), 스캔 정합 오차(scan-matching error), 시각 추적 불안정성(visual tracking instability), 관성 적분 문제(inertial integration problem)를 확인할 수 있다. 긴 구간의 RTE는 더 큰 궤적 구간에서 누적된 드리프트를 나타낸다. 여러 구간 길이를 평가하면 단기 운동 불확실성이 장기적인 위치추정 드리프트로 어떻게 성장하는지를 파악할 수 있다.

상대 병진 오차(relative translational error)는 미터(meter) 단위로 표현할 수 있지만 이동 거리로 정규화(normalization)하면 서로 다른 실험을 보다 의미 있게 비교할 수 있다. 예를 들어 시스템은 구간 길이에 대한 백분율로 병진 드리프트를 표현하거나 100미터 이동당 몇 미터의 위치 오차가 발생하는지를 보고할 수 있다. 회전 드리프트(rotational drift) 역시 응용 분야와 벤치마크 규칙에 따라 미터당 각도(degrees per meter) 또는 100미터당 각도와 같이 거리 기준으로 정규화할 수 있다.

따라서 ATE와 RTE는 서로 보완적인 특성을 설명한다. ATE는 추정 궤적이 올바른 전역 궤적으로부터 얼마나 떨어져 있는지를 나타내고, RTE는 서로 떨어진 두 자세 사이의 운동을 얼마나 정확하게 추정했는지를 나타낸다. 우수한 국부 오도메트리(local odometry)를 가진 시스템도 전역 보정이 없다면 단기 RTE는 작지만 ATE는 지속적으로 증가할 수 있다. 반대로 주기적인 지도 또는 GNSS 보정은 상당한 국부 상대 운동 오차가 존재하더라도 ATE를 낮게 유지할 수 있다.

이러한 차이는 동시적 위치추정 및 지도작성(SLAM)을 평가할 때 특히 중요하다. 루프 폐쇄(loop closure)는 이전 자세에 누적된 드리프트를 재분배하여 전역 궤적 오차(global trajectory error)를 크게 감소시킬 수 있다. 따라서 전역 최적화(global optimization) 이후 측정된 ATE는 매우 작아질 수 있지만 그 이전의 오도메트리에는 상당한 국부 오차가 누적되어 있었을 수 있다. ATE와 함께 RTE를 보고하면 정확한 증분 운동 추정(incremental motion estimation)과 성공적인 전역 궤적 보정(global trajectory correction)을 구분할 수 있다.

병진과 회전은 서로 다른 고장 메커니즘(failure mechanism)을 나타내므로 일반적으로 별도로 평가해야 한다. 로봇은 정확한 위치를 유지하면서 자세 오차를 누적할 수도 있고 그 반대의 상황도 발생할 수 있다. 특히 헤딩 오차(heading error)는 작은 각도 오차라도 이후의 전진 운동에서 큰 횡방향 위치 오차(lateral position error)를 발생시킬 수 있기 때문에 이동 로봇에서 매우 중요하다. 3차원 시스템에서는 롤(roll), 피치(pitch), 요(yaw), 수직 위치(vertical position)를 별도로 분석해야 할 수도 있다.

종단점 오차(endpoint error)는 간단하지만 제한적인 또 다른 평가 지표이다. 이는 궤적 마지막 지점에서 추정 자세와 실제 자세 사이의 차이를 측정한다. 폐쇄 경로 시험(closed-loop test)이나 빠른 상태 확인에는 유용하지만 경로 중간에 발생한 오차를 반영하지 못한다. 궤적이 중간에서 크게 벗어났다가 마지막에 올바른 위치 근처로 돌아올 수도 있기 때문이다. 따라서 종단점 오차는 ATE와 RTE 같은 전체 궤적 지표를 대체하기보다 보완적으로 사용해야 한다.

루프 폐쇄 오차(loop-closure error)는 로봇이 이전에 방문했던 물리적 위치로 다시 돌아오는 경로에서 유용하다. 추정된 시작 자세와 종료 자세 사이의 차이는 루프 보정 이전에 누적된 불일치(inconsistency)를 나타낸다. 병진 및 회전 폐쇄 오차는 체계적인 오도메트리 바이어스(systematic odometry bias)와 장기 드리프트를 확인하는 데 사용할 수 있다. 그러나 폐쇄 오차가 작다고 해서 궤적의 중간 구간까지 정확하게 추정되었다는 것을 의미하지는 않는다.

위치추정 오차는 비가우시안(non-Gaussian) 특성을 가지며 이상치(outlier)를 포함하는 경우가 많기 때문에 통계적 요약값(statistical summary)은 신중하게 해석해야 한다. RMSE는 큰 실패에 높은 가중치를 부여하는 반면 중앙값 오차(median error)는 소수의 심각한 오류가 존재할 때 일반적인 성능을 보다 잘 나타낼 수 있다. 95번째 또는 99번째 백분위수는 안전과 관련된 꼬리 영역 거동(tail behavior)을 표현할 수 있다. 최대 오차(maximum error)는 관측된 최악의 사건을 나타내지만 단일 측정 오류나 기준값 이상에 민감할 수 있다.

위치추정 평가에는 신뢰할 수 있는 기준값(ground truth)도 필요하다. 실내 실험에서는 모션 캡처 시스템(motion-capture system), 레이저 트래커(laser tracker), 측량된 랜드마크(surveyed landmark), 고품질 기준 지도(reference map)를 사용할 수 있다. 야외 시험에서는 측량급 GNSS/INS(survey-grade GNSS/INS), 토털 스테이션(total station), 기타 독립적으로 검증된 위치 측정 시스템을 사용할 수 있다. 기준값의 불확실성은 측정하려는 위치추정 오차보다 충분히 작아야 하며, 그렇지 않으면 알고리즘이 아니라 기준 시스템의 잡음을 평가하게 된다.

기준값은 순환 평가(circular evaluation)를 방지할 수 있도록 충분히 독립적이어야 한다. 위치추정 알고리즘과 기준 궤적이 동일한 GNSS 수신기나 지도 제약조건에 크게 의존하면 두 시스템의 오차가 서로 상관될 수 있다. 이 경우 두 결과가 일치하더라도 실제 정확도를 과대평가할 가능성이 있다. 독립적인 센싱(independent sensing), 더 높은 등급의 기준 장비(reference equipment), 정밀하게 측량된 환경을 사용하면 위치추정 성능에 대해 더 강한 검증 근거를 확보할 수 있다.

시간 동기화(time synchronization)는 평가 오차를 발생시키는 주요 원인 중 하나이다. 기준 궤적과 추정 궤적 사이에 작은 시간 오프셋만 존재해도 움직이는 로봇은 위치추정이 정확함에도 공간적으로 이동된 것처럼 나타난다. 이로 인해 발생하는 오차는 차량 속도와 각속도(angular velocity)가 증가할수록 커진다. 따라서 ATE나 RTE를 해석하기 전에 정확한 하드웨어 타임스탬프(hardware timestamp), 클록 동기화(clock synchronization), 지연 보상(latency compensation), 시간 정렬 검증(temporal alignment validation)이 필수적이다.

궤적 샘플링(trajectory sampling) 역시 평가 지표에 영향을 미친다. 조밀한 추정 궤적을 희소한 기준값과 비교하려면 보간이 필요할 수 있으며, 불균일한 샘플링(uneven sampling)은 특정 궤적 구간이 전체 통계에 지나치게 큰 영향을 주게 할 수 있다. 이동 로봇에서는 거리 기반 재표본추출(distance-based resampling)을 통해 보다 균형 잡힌 평가를 수행할 수 있다. 실험의 재현성과 공정한 비교를 위해 사용된 샘플링 및 보간 전략을 명확하게 기록해야 한다.

평가는 하나의 유리한 궤적이 아니라 실제 운용을 대표하는 조건을 포함해야 한다. 직선 주행, 급회전, 저속 기동, 고속 이동, 경사면, 반복적인 환경, 개방 공간, 동적 장애물, 휠 슬립, GNSS 성능 저하, 센서 가림(sensor occlusion)은 서로 다른 위치추정 약점을 드러낼 수 있다. 야외 자율이동로봇(Outdoor AMR)은 포장도로, 자갈길, 식생 지역, 건물 주변, 터널, 그리고 서로 다른 위치추정 모드(localization mode) 사이의 전환 구간에서도 추가적으로 시험해야 한다.

구간 기반 분석(segment-based analysis)을 사용하면 위치추정 성능이 저하되는 지점을 식별할 수 있다. 긴 경로를 거리, 지형, 환경 유형, 센서 사용 가능 여부(sensor availability), 위치추정 모드에 따라 구분한 뒤 각 구간의 ATE와 RTE를 분석할 수 있다. 이를 통해 오차가 추정기의 일반적인 한계에서 발생하는지, GNSS 다중경로(GNSS multipath), 라이다 퇴화(LiDAR degeneracy), 시각 성능 저하(visual degradation), 심각한 휠 슬립과 같은 특정 사건에서 발생하는지를 구분할 수 있다.

정확도(accuracy)는 정밀도(precision) 및 일관성(consistency)과도 구분해야 한다. 정확도는 기준값에 얼마나 가까운지를 의미하고, 정밀도는 반복성(repeatability) 또는 분산 정도를 의미하며, 일관성은 추정기가 보고하는 불확실성이 실제 오차와 통계적으로 일치하는지를 나타낸다. 위치추정 시스템은 반복성은 높지만 바이어스가 존재할 수도 있고, 평균적으로는 정확하지만 공분산이 잘못 보정되어 있을 수도 있다. 따라서 완전한 평가는 궤적 오차와 불확실성 품질을 함께 고려해야 한다.

어떤 수준의 오차가 허용 가능한지는 응용 요구사항(application requirement)에 따라 결정된다. 좁은 통로를 이동하는 창고 자율이동로봇(warehouse AMR)은 센티미터 수준의 횡방향 정확도가 필요할 수 있지만, 개방된 지형을 운행하는 대형 야외 플랫폼은 더 큰 절대 위치 오차를 허용할 수 있다. 도킹(docking), 매니퓰레이션(manipulation), 군집 주행(convoy operation), 고속 내비게이션(high-speed navigation)은 서로 다른 요구사항을 가진다. 위치추정 지표는 기능적 안전 여유(functional safety margin)와 임무 요구사항(mission requirement)에 연결될 때 실질적인 엔지니어링 도구가 된다.

다중 로봇 시스템(multi-robot system)에서는 로봇 사이의 상대 위치추정(relative localization)을 추가적으로 평가할 수 있다. 각각의 로봇이 어느 정도의 전역 오차를 가지고 있더라도 정확한 상대 자세(relative pose)를 확보하면 편대 제어(formation control), 협력 인지(cooperative perception), 충돌 회피(collision avoidance)를 수행할 수 있다. 반대로 개별 로봇의 위치가 정확하더라도 서로 일치하지 않는 전역 좌표계를 사용하면 올바른 협업이 어려울 수 있다. 따라서 전역 ATE, 국부 RTE, 로봇 간 상대 오차(inter-robot relative error)는 플릿 위치추정(fleet localization)의 서로 다른 계층을 평가할 수 있다.

엄격한 벤치마크(rigorous benchmark)는 지표 값을 올바르게 해석할 수 있도록 충분한 정보를 함께 제공해야 한다. 여기에는 궤적 시간과 길이, 센서 구성(sensor configuration), 기준값 획득 방법, 좌표 정렬 방법(coordinate alignment method), 스케일 처리(scale treatment), 시간 연관 방식, ATE 통계값, RTE 구간, 병진 및 회전 단위, 관련 환경 조건이 포함된다. 이러한 정보가 없으면 겉으로 동일한 두 수치가 실제로는 근본적으로 다른 수준의 위치추정 성능을 나타낼 수 있다.

궁극적으로 절대 궤적 오차(ATE)와 상대 궤적 오차(RTE)는 위치추정 품질(localization quality)을 서로 보완적인 관점에서 평가한다. ATE는 기준값에 대한 전역 궤적의 일관성(global trajectory consistency)을 측정하고, RTE는 상대 운동의 정확성과 국부 드리프트의 증가를 측정한다. 회전 오차(rotational error), 백분위수 통계(percentile statistics), 불확실성 일관성(uncertainty consistency), 대표적인 시나리오 시험과 함께 사용하면 실제 운용 조건에서 위치추정 알고리즘을 비교하고 자율 로봇의 성능을 검증하기 위한 체계적인 기반을 제공한다.

## 01.08. Localization Failure Modes Kidnapped Robot Problem

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

위치추정 실패(localization failure)는 로봇의 추정 자세(estimated pose)가 신뢰할 수 있는 운용에 필요한 정확도 수준으로 실제 위치와 방향을 더 이상 나타내지 못할 때 발생한다. 실패는 누적된 드리프트(accumulated drift)를 통해 점진적으로 발생하거나 잘못된 측정, 지도 연관(map association), 외부 교란(external disturbance) 이후 갑작스럽게 발생할 수 있다. 내비게이션(navigation), 경로 계획(planning), 장애물 해석(obstacle interpretation)이 자세에 의존하므로 위치추정 실패는 자율 시스템 전체의 오류로 빠르게 확산될 수 있다.

위치추정 시스템(localization system)은 수치적으로 유효한 자세를 계속 출력하면서도 실패할 수 있다. 이러한 상태는 하위 모듈(downstream module)이 추정값이 잘못되었다는 사실을 즉시 인식하지 못할 수 있기 때문에 특히 위험하다. 궤적은 계속 부드럽게 나타나고 공분산(covariance)은 작게 유지되며 센서 갱신도 정상적으로 계속되는 것처럼 보이지만 실제로 로봇은 잘못된 위치에 있을 수 있다. 따라서 잘못된 신뢰도(incorrect confidence)를 검출하는 것은 자세 자체를 추정하는 것만큼 중요하다.

점진적 드리프트(gradual drift)는 가장 일반적인 고장 형태 중 하나이다. 휠 오도메트리(wheel odometry)는 슬립(slip), 타이어 변형(tire deformation), 보정 불확실성(calibration uncertainty), 불완전한 차량 모델로 인한 오차를 누적한다. IMU 적분은 바이어스(bias)와 잡음을 누적하며, 시각 및 라이다 오도메트리(visual and LiDAR odometry) 역시 작은 정합 오차(registration error)를 누적할 수 있다. 신뢰할 수 있는 절대 기준 또는 지도 기준 보정(map-referenced correction)이 없다면 이러한 증분 오차는 추정 궤적을 로봇의 실제 궤적으로부터 점차 분리시킨다.

갑작스러운 위치추정 점프(localization jump)는 다른 유형의 고장 형태를 나타낸다. 잘못된 스캔 정합(scan matching), 잘못된 시각 대응(false visual correspondence), 손상된 GNSS 관측값(corrupted GNSS observation), 잘못된 루프 폐쇄(erroneous loop closure)는 추정 자세를 갑자기 잘못된 위치로 이동시킬 수 있다. 실제 로봇은 이에 대응하는 물리적 이동을 하지 않았기 때문에 이러한 점프는 제어기(controller)에 특히 위험할 수 있다. 불연속성 모니터링(discontinuity monitoring)과 독립적인 운동 추정값 사이의 일관성 검사를 통해 이러한 사건을 식별할 수 있다.

지각적 앨리어싱(perceptual aliasing)은 서로 다른 물리적 위치가 유사한 센서 관측값을 생성할 때 발생한다. 반복되는 창고 통로, 동일한 형태의 복도, 반복적인 건물 외벽, 주차 공간, 규칙적으로 배치된 구조물은 이러한 모호성을 만들 수 있다. 위치추정 알고리즘은 현재 관측값을 지도의 잘못된 영역과 높은 신뢰도로 연관시킬 수 있다. 따라서 기하학적 유사성(geometric similarity)은 지도 기반 위치추정(map-based localization)과 루프 폐쇄 시스템(loop-closure system)에서 잘못된 수렴(false convergence)을 발생시키는 주요 원인이다.

특징이 부족한 환경(feature-poor environment)은 반대의 문제를 발생시킨다. 길고 단조로운 복도, 개방된 들판, 매끄러운 벽, 넓은 포장 공간 또는 기하학적 변화가 제한적인 환경에서는 모든 자세 차원을 제약하기에 충분한 정보를 얻지 못할 수 있다. 라이다 스캔 정합은 퇴화(degenerate)될 수 있으며 시각 위치추정은 텍스처(texture)나 조명(illumination)이 부족할 때 실패할 수 있다. 추정기는 관측 가능성이 낮은 방향(weakly observable direction)의 불확실성이 증가하는 동안에도 운동 전파를 계속할 수 있다.

동적 환경(dynamic environment)은 저장된 지도가 현재 관측되는 실제 환경을 나타낸다는 가정을 위반한다. 사람, 차량, 팔레트, 문, 임시 구조물, 식생(vegetation), 건설 장비, 재배치된 가구 등이 센서 측정값의 상당 부분을 차지할 수 있다. 동적 객체(dynamic object)를 영구적인 지도 특징으로 해석하면 스캔 정합이나 시각 연관(visual association)이 편향된 자세 추정값을 생성할 수 있다. 강건한 위치추정(robust localization)은 지속적인 환경 구조와 일시적인 변화를 구별할 수 있어야 한다.

환경 조건(environmental condition)은 센서 성능을 직접적으로 저하시킬 수 있다. 카메라는 어둠, 눈부심(glare), 그림자, 비, 안개, 먼지, 렌즈 오염(lens contamination), 모션 블러(motion blur)의 영향을 받는다. 라이다 측정값은 강수, 반사 표면, 먼지, 불리한 기하학적 구조의 영향을 받을 수 있다. GNSS는 건물, 나무, 교량, 터널 및 기타 구조물 주변에서 신호 차단과 다중경로(multipath)로 인해 성능이 저하될 수 있다. 따라서 위치추정 신뢰성은 고정된 센서 사양이 아니라 시간에 따라 변화하는 특성으로 평가해야 한다.

기계적 영향(mechanical effect) 역시 위치추정 실패에 기여한다. 휠 마모(wheel wear), 타이어 공기압 변화, 서스펜션 운동(suspension movement), 페이로드(payload) 변화, 엔코더 고장(encoder fault), 조향 백래시(steering backlash), 섀시 변형(chassis deformation), 휠 슬립은 측정된 액추에이터 운동과 실제 차량 변위 사이의 관계를 변화시킨다. 플랫폼의 상태가 변화함에 따라 기존에 보정된 오도메트리 모델도 점차 부정확해질 수 있으므로 장기적인 자율 운용에서는 상태 모니터링(health monitoring)과 주기적인 재보정(recalibration)이 중요하다.

시간 관련 오류(timing error)는 개별 센서가 정확하더라도 위치추정 고장을 발생시킬 수 있다. 잘못된 타임스탬프(timestamp), 네트워크 지연(network latency), 클록 드리프트(clock drift), 버퍼링(buffering), 동기화 오류(synchronization error)는 관측값을 잘못된 로봇 상태와 연결시킨다. 빠른 병진 또는 회전 운동 중에는 작은 시간 오프셋도 큰 공간적 불일치(spatial inconsistency)를 발생시킬 수 있다. 따라서 다중 센서 위치추정(multi-sensor localization)은 공간 보정(spatial calibration)만큼 정확한 시간 아키텍처(timing architecture)에 의존한다.

센서 사이의 외부 파라미터 보정 오차(extrinsic calibration error)는 체계적인 불일치(systematic inconsistency)를 발생시킨다. IMU, 라이다, 카메라, GNSS 안테나, 로봇 베이스 사이의 변환 관계가 잘못되어 있으면 측정값을 하나의 공통 물리 좌표계에서 올바르게 해석할 수 없다. 작은 각도 보정 오차도 긴 센싱 거리에서는 큰 오차로 확대될 수 있다. 유지보수나 충돌 이후 발생한 기계적 이동은 이전에 정확했던 보정값을 무효화하고 지속적인 위치추정 바이어스를 발생시킬 수 있다.

지도 오류(map error)는 또 다른 중요한 실패 원인이다. 지도에는 기하학적 왜곡(geometric distortion), 오래되어 변경된 구조물, 잘못된 좌표 기준(coordinate reference), 일관되지 않은 해상도, 최초 지도작성 과정에서 발생한 아티팩트(artifact)가 포함될 수 있다. 이러한 지도를 이용한 위치추정은 겉으로는 안정적이지만 편향된 자세를 생성할 수 있다. 따라서 장기간 운용되는 자율 시스템에는 지도 검증(map validation), 버전 관리(version control), 변화 감지(change detection), 기존 지도를 더 이상 신뢰할 수 없는 시점을 판단하는 메커니즘이 필요하다.

잘못된 초기화(incorrect initialization)는 위치추정 알고리즘이 잘못된 가설로 수렴하게 만들 수 있다. EKF 기반 시스템과 같은 국부 추정기(local estimator)는 일반적으로 초기 상태가 실제 상태에 충분히 가깝다고 가정한다. 스캔 정합 역시 초기 자세가 실제 위치로부터 너무 멀리 떨어져 있으면 국부 최솟값(local minimum)에 수렴할 수 있다. 초기 불확실성이 물리적으로 서로 떨어진 여러 영역에 걸쳐 존재한다면 전역 위치추정(global localization) 방법이 필요하다.

잘못된 수렴(false convergence)은 추정기가 잘못된 자세에 대해 높은 신뢰도를 가지는 상태이다. 이는 단순한 불확실성보다 더 심각한 문제인데, 시스템이 다른 대안 가설(alternative hypothesis)을 탐색하지 않을 수 있기 때문이다. 파티클 필터(particle filter)는 파티클 다양성이 부족하면 올바른 가설을 잃을 수 있으며, 최적화 기반 시스템(optimization-based system)은 잘못된 연관을 받아들일 수 있다. 따라서 신뢰도는 공분산이나 파티클 집중도만으로 평가해서는 안 되며 관측 일관성(observation consistency)과 가설 품질(hypothesis quality)을 함께 고려해야 한다.

납치된 로봇 문제(kidnapped robot problem)는 치명적인 위치추정 상실(catastrophic localization loss)을 대표하는 전형적인 문제이다. 로봇이 현재 위치에서 다른 위치로 물리적으로 이동되었지만 위치추정 시스템에는 이에 대응하는 운동 정보가 전달되지 않은 상황을 가정한다. 내부 믿음(belief)은 로봇이 실제로 다른 위치에 있음에도 기존 자세 주변에 계속 집중되어 있다. 이에 따라 센서 관측값은 기존 믿음으로부터 예측된 관측값과 갑작스럽게 불일치하게 된다.

명칭은 로봇을 실제로 들어서 옮기는 상황을 연상시키지만, 추정 상태가 실제 상태와 심각하게 분리되는 모든 상황에서 동일한 문제가 발생할 수 있다. 심각한 휠 슬립, 잘못된 지도 전환(map transition), 잘못된 루프 폐쇄, 추정기 재설정(estimator reset), GNSS 점프, 기계적인 운반, 소프트웨어 오류는 모두 이와 동등한 상태를 만들 수 있다. 따라서 납치된 로봇 문제는 큰 위치추정 오차로부터 자율적으로 복구해야 한다는 보다 일반적인 요구사항을 나타낸다.

일반적인 국부 가우시안 추정기(local Gaussian estimator)는 하나의 지배적인 상태 추정값 주변에서 불확실성을 표현하기 때문에 이러한 상태로부터 복구하기 어렵다. 실제 자세가 해당 분포로부터 멀리 벗어나 있다면 일반적인 측정 보정만으로 추정값을 실제 위치까지 이동시키기 어려울 수 있다. 공분산을 증가시키면 불확실성 범위를 확장할 수 있지만 서로 멀리 떨어진 여러 자세 가설을 자연스럽게 표현하지는 못한다. 따라서 전역 재위치추정(global relocalization) 메커니즘이 필요한 경우가 많다.

파티클 필터는 서로 분리된 여러 가설을 표현할 수 있기 때문에 납치된 로봇 문제의 복구에 적합하다. 현재 파티클 집합 주변에서 관측 우도(observation likelihood)가 지속적으로 낮아지면 지도의 다른 위치에 새로운 파티클을 주입할 수 있다. 이러한 파티클 중 일부가 센서 관측을 더 잘 설명하면 이후의 갱신 과정에서 가중치가 증가하고 파티클 집합은 로봇의 실제 위치 주변으로 이동할 수 있다.

무작위 파티클 주입(random particle injection)은 복구 속도와 정상 추적 안정성(normal tracking stability) 사이에서 균형을 유지해야 한다. 지나치게 많은 전역 파티클을 지속적으로 추가하면 계산 자원이 낭비되고 국부 정밀도가 저하될 수 있지만 너무 적게 추가하면 복구 시간이 지나치게 길어질 수 있다. 적응형 방법(adaptive method)은 센서 우도가 저하될 때 탐색을 증가시키고 위치추정 신뢰도가 다시 확보되면 이를 감소시킨다. 이를 통해 추정기는 관측 증거에 따라 추적 모드와 복구 모드 사이를 전환할 수 있다.

고장 검출(failure detection)은 복구보다 먼저 또는 복구와 동시에 수행되어야 한다. 유용한 지표에는 예상보다 낮은 관측 우도, 큰 혁신(innovation), 일관되지 않은 센서 잔차(sensor residual), 빠른 공분산 증가, 오도메트리와 지도 위치추정 사이의 불일치, 물리적으로 설명하기 어려운 자세 점프, 지속적인 지도 불일치(map mismatch)가 포함된다. 하나의 지표만으로 모든 상황을 신뢰성 있게 판단할 수 없으므로 실제 시스템에서는 여러 독립적인 신호를 결합하여 위치추정 건전성(localization health)을 평가하는 경우가 많다.

센서 간 일관성(cross-sensor consistency)은 특히 중요한 판단 근거를 제공한다. 휠 오도메트리, IMU, GNSS, 라이다, 시각 위치추정은 서로 다른 고장 메커니즘을 가진다. 라이다 위치추정이 큰 변위를 보고하지만 IMU와 휠 측정값에서는 거의 운동이 감지되지 않는다면 해당 관측값은 신중하게 처리해야 한다. 반대로 독립적인 여러 센싱 방식이 서로 일치하면 하나의 센서가 일시적으로 불안정한 상황에서도 위치추정 신뢰도를 높일 수 있다.

혁신 게이팅(innovation gating)과 이상치 제거(outlier rejection)는 하나의 잘못된 관측값이 상태 추정값을 손상시키는 것을 방지할 수 있다. 예측된 불확실성과 비교하여 통계적으로 일관되지 않은 잔차를 가진 측정값은 거부하거나 가중치를 낮출 수 있다. 그러나 지나친 측정 거부 역시 위험하다. 추정기의 예측 자체가 잘못된 상태가 된 이후에도 실제로 올바른 관측값을 계속 거부할 수 있기 때문이다. 강건한 위치추정은 센서 이상치와 현재 상태 가설 자체가 실패했다는 증거를 구별할 수 있어야 한다.

따라서 위치추정 건전성은 단순한 유효 또는 무효(valid-or-invalid) 플래그가 아니라 명시적인 상태로 표현하는 것이 바람직하다. 시스템은 정상 추적(nominal tracking), 성능 저하 추적(degraded tracking), 불확실(uncertain), 재위치추정 중(relocalizing), 위치 상실(lost)과 같은 상태로 구분할 수 있다. 각각의 상태는 경로 계획과 제어에서 서로 다른 동작을 유발할 수 있다. 신뢰도가 감소하면 로봇은 속도를 낮추고 장애물 안전 여유를 증가시키며 복잡한 기동을 제한하거나 안전하게 정지하고 능동적 재위치추정(active relocalization)을 시작할 수 있다.

능동적 재위치추정(active relocalization)은 더 많은 정보를 가진 관측값을 얻기 위해 의도적으로 로봇을 움직이는 방법이다. 현재 시점에서 여러 지도 위치가 비슷하게 보이는 경우 로봇을 회전시키거나 특징적인 구조물 방향으로 이동하거나 관측 시점을 변경하여 모호성을 줄일 수 있다. 따라서 위치추정과 경로 계획은 서로 연결되어 있으며, 목적지에 가장 빠르게 도달하는 운동이 위치추정을 잃기 직전의 상황에서 자세 불확실성을 감소시키는 최적의 운동과 항상 동일한 것은 아니다.

복구 과정에서는 여러 수준의 공간 탐색(spatial search)을 사용할 수 있다. 작은 국부 탐색(local search)은 중간 수준의 드리프트를 보정할 수 있고, 더 넓은 지역 탐색(regional search)은 큰 오차를 복구할 수 있으며, 전체 지도 전역 위치추정(full-map global localization)은 자세를 완전히 상실한 상황을 처리할 수 있다. 계층적 복구(hierarchical recovery)는 모든 불일치가 발생할 때마다 즉시 전체 환경을 탐색하지 않아도 되므로 계산량을 줄일 수 있다. GNSS, 임무 상황(mission context), 위상 정보(topology), 주행 가능한 영역 정보를 이용하여 탐색 영역을 제한할 수도 있다.

야외 자율이동로봇(Outdoor AMR)에서는 운용 조건이 빠르게 변화할 수 있기 때문에 위치추정 실패 관리가 특히 중요하다. 로봇은 RTK-GNSS가 안정적인 개방된 하늘 환경에서 도심 협곡(urban canyon), 나무가 우거진 도로, 창고 입구, 터널로 이동할 수 있다. 자갈, 진흙, 눈, 경사면에서는 휠 슬립이 증가할 수 있으며 동시에 카메라와 라이다 품질도 변화할 수 있다. 강건한 자율주행(robust autonomy)을 위해서는 하나의 센싱 방식이 영구적으로 신뢰할 수 있다고 가정하기보다 여러 위치추정 정보원 사이를 안정적으로 전환해야 한다.

실제적인 아키텍처에서는 연속적인 국부 추정(continuous local estimation), 독립적인 전역 위치추정(independent global localization), 건전성 감독(health supervision)을 결합할 수 있다. IMU와 휠 오도메트리는 오도메트리 좌표계(odom frame)에서 부드러운 고주기 운동을 유지하고, GNSS, 라이다, 비전 또는 지도 정합(map matching)은 전역 자세를 제약한다. 위치추정 감독기(localization supervisor)는 일관성을 모니터링하고 전역 보정을 언제 신뢰할지, 언제 불확실성을 증가시킬지, 언제 재위치추정 또는 안전 대응을 수행할지를 결정한다.

위치추정 복구 시험(localization recovery testing)은 정상적인 정확도만 평가하는 것이 아니라 의도적으로 실패 조건을 발생시켜야 한다. 잘못된 초기화, 일시적인 센서 상실, GNSS 점프, 인위적인 지도 불일치, 심각한 휠 슬립, 반복적인 환경, 잘못된 연관, 타임스탬프 교란, 로봇의 의도적인 위치 이동 등을 시험에 포함해야 한다. 복구 시간(recovery time), 복구 이전까지의 이동 거리, 잘못된 복구율(false-recovery rate), 복구 이후의 자세 오차, 불확실한 상태에서의 안전 동작을 측정해야 한다.

궁극적으로 신뢰성 높은 위치추정(reliable localization)을 위해서는 유리한 조건에서 정확한 자세를 생성하는 능력뿐만 아니라 현재 자세 추정값이 잘못되었을 가능성을 스스로 인식할 수 있어야 한다. 드리프트, 지각적 모호성, 센서 성능 저하, 지도 변화, 보정 오류, 시간 오류, 치명적인 상태 이탈은 모두 위치추정의 기본 가정을 무너뜨릴 수 있다. 납치된 로봇 문제는 자율 로봇이 위치추정 상실을 감지하고, 안전성을 유지하며, 대체 가설을 탐색하고, 외부 개입 없이 신뢰할 수 있는 자세를 복구해야 한다는 핵심 요구사항을 대표한다.

## 01.09. Indoor vs Outdoor Localization Requirements Comparison

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

실내 및 실외 위치추정(indoor and outdoor localization)은 자율 운용(autonomous operation)에 필요한 충분한 정확도(accuracy), 연속성(continuity), 신뢰도(confidence)를 갖추어 로봇의 위치(position)와 자세(orientation)를 추정한다는 동일한 기본 목표를 가진다. 그러나 센싱 조건(sensing condition), 환경 규모(environmental scale), 차량 동역학(vehicle dynamics), 지도 특성(map characteristics), 사용 가능한 전역 기준(global reference)이 영역에 따라 달라지기 때문에 엔지니어링 요구사항은 상당한 차이를 보인다. 따라서 위치추정 아키텍처(localization architecture)는 하나의 범용 센서 구성보다는 실제 운용 조건을 중심으로 설계해야 한다.

실내 이동 로봇(indoor mobile robot)은 일반적으로 창고, 공장, 병원, 사무실, 연구실 또는 물류 시설과 같이 물리적으로 제한된 환경에서 운용된다. 운용 영역은 개별 공간에서 대규모 산업용 건물까지 다양할 수 있지만 일반적으로 벽, 기둥, 랙(rack), 문, 복도, 장비와 같은 구조화된 특징(structured feature)을 포함한다. 이러한 구조물은 라이다(LiDAR), 카메라(camera), 지도 기반 위치추정(map-based localization)을 지원할 수 있는 기하학적 제약(geometric constraint)을 제공한다.

실외 로봇(outdoor robot)은 도로, 산업 현장, 캠퍼스, 항만, 건설 지역, 농경지, 포장 및 비포장 혼합 지형과 같이 상대적으로 제약이 적은 환경에서 운용된다. 경로는 수 킬로미터 이상 확장될 수 있으며 주변 구조물이 제한적인 긴 구간이 존재할 수 있다. 따라서 위치추정은 훨씬 큰 공간 규모에서 안정성을 유지하고 개방된 하늘, 건물 주변, 식생 지역, 터널 및 기타 다양한 센싱 조건 사이의 전환 과정에서도 연속성을 유지해야 한다.

주요 차이점 중 하나는 위성항법시스템(GNSS)의 사용 가능 여부이다. 일반적인 위성 위치측정(satellite positioning)은 건물 구조물이 위성 신호를 감쇠시키고 반사하기 때문에 실내에서는 대부분 사용할 수 없거나 신뢰성이 낮다. 따라서 실내 위치추정은 주로 온보드 상대 센싱(onboard relative sensing), 지역 인프라(local infrastructure), 지도에 의존한다. 휠 오도메트리(wheel odometry), 관성측정장치(IMU), 라이다 위치추정, 시각 위치추정(visual localization), 기준 마커(fiducial marker), 초광대역(UWB) 및 기타 지역 기준을 요구 정확도와 시설 구성에 따라 결합할 수 있다.

실외 환경에서는 GNSS를 중요한 전역 기준(global reference)으로 사용할 수 있다. 실시간 이동측위 위성항법(RTK-GNSS)은 위성 가시성(satellite visibility), 보정 데이터(correction data), 신호 기하(signal geometry)가 양호한 경우 높은 정확도의 위치 정보를 제공할 수 있다. 그러나 GNSS를 항상 신뢰할 수 있는 센서로 가정해서는 안 된다. 건물, 나무, 교량, 컨테이너, 산업 구조물, 터널, 전파 간섭(interference), 다중경로(multipath)는 정확도를 저하시키거나 GNSS를 완전히 사용할 수 없게 만들 수 있으므로 위성 위치측정을 보완하거나 대체할 수 있는 독립적인 위치추정 방법이 필요하다.

실내 위치추정은 반복성(repeatability)과 정밀한 횡방향 위치 결정(lateral positioning)을 중요하게 고려하는 경우가 많다. 창고 로봇은 좁은 통로를 주행하고, 선반과 정렬하며, 엘리베이터에 진입하고, 문을 통과하거나 충전 및 물류 처리 스테이션에 도킹해야 할 수 있다. 운용 영역이 작더라도 센티미터 수준의 국부 정확도(local accuracy)가 중요할 수 있다. 헤딩 오차(heading error)는 이후 전진 운동에서 횡방향 변위를 발생시키므로 안정적인 자세 추정 역시 중요하다.

실외 위치추정 요구사항은 속도(speed), 정지 거리(stopping distance), 경로 형상(route geometry), 안전 여유(safety margin)와 밀접하게 관련된다. 개방된 공간에서 저속으로 이동할 때 허용할 수 있는 위치 오차도 도로 경계, 보행자, 기반 시설 또는 다른 차량 근처에서는 허용하기 어려울 수 있다. 따라서 실외 위치추정에는 절대 정확도뿐만 아니라 예측 가능한 오차 범위(error bound), 높은 가용성(availability), 고장 검출(fault detection), 동적 내비게이션을 지원할 수 있는 충분한 갱신 주기(update rate)가 필요하다.

실내 환경은 2차원 라이다 위치추정(2D LiDAR localization)에 유용한 강한 기하학적 구조를 제공하는 경우가 많다. 벽, 랙 모서리, 기둥, 복도는 점유 지도(occupancy map) 또는 특징 지도(feature map)와 정합할 수 있는 반복 가능한 스캔 패턴(scan pattern)을 생성한다. 그러나 반복적인 구조를 가진 창고에서는 여러 통로가 기하학적으로 유사하게 보이는 지각적 앨리어싱(perceptual aliasing)이 발생할 수 있다. 따라서 기하학적 특징이 풍부하다고 해서 항상 전역적으로 고유한 위치추정(global localization)이 보장되는 것은 아니다.

실외 라이다 위치추정은 다른 형태의 문제를 가진다. 건물, 연석(curb), 기둥, 방호벽 및 기타 지속적인 구조물은 유용한 지도 특징을 제공하지만 개방된 도로나 들판에서는 주변 기하 구조가 충분하지 않을 수 있다. 식생은 형태가 변하고 차량은 이동하며 긴 센싱 거리는 보정 오차(calibration error)를 증폭시킨다. 따라서 실외에서는 평면적인 실내 환경보다 3차원 라이다(3D LiDAR)와 보다 풍부한 기하학적 지도(geometric map)가 중요해지는 경우가 많다.

카메라 기반 위치추정(camera-based localization) 역시 두 환경에서 서로 다른 특성을 가진다. 실내 조명은 인공적이고 비교적 제어 가능하지만 반사되는 바닥, 반복적인 텍스처(texture), 제한된 시야각(field of view), 저조도 영역이 문제를 발생시킬 수 있다. 실외 비전(outdoor vision)은 햇빛, 그림자, 야간 운용, 날씨, 계절 변화, 눈부심(glare), 큰 밝기 변화에 대응해야 한다. 카메라는 풍부한 의미론적 정보(semantic information)를 제공할 수 있지만 신뢰성은 환경 조건에 따라 크게 달라진다.

휠 오도메트리는 외부 인프라가 없어도 높은 주기의 상대 운동 정보를 제공하기 때문에 실내와 실외 모두에서 유용하다. 실내에서는 평탄한 바닥과 예측 가능한 접지력(traction)으로 인해 비교적 안정적인 오도메트리를 얻을 수 있다. 실외에서는 자갈, 진흙, 잔디, 경사면, 연석, 포트홀(pothole), 느슨한 노면, 급격한 조향으로 인해 훨씬 큰 슬립이 발생할 수 있다. 따라서 운동 모델 불확실성(motion-model uncertainty)은 고정된 값으로 유지하기보다 지형과 차량 거동에 따라 적응적으로 변화해야 한다.

관성측정장치(IMU)는 실내와 실외 위치추정 모두에서 높은 주기의 회전 및 가속도 정보를 제공한다. 단기적인 연속성(short-term continuity)은 외부 기준을 일시적으로 사용할 수 없는 상황에서 특히 중요하다. 그러나 관성 측정값은 적분 과정에서 바이어스를 누적하므로 독립적으로 장기간 안정적인 전역 위치를 제공할 수 없다. 진동, 지형 충격, 서스펜션 운동, 높은 속도를 경험하는 실외 플랫폼은 더욱 신중한 장착, 보정, 동역학 모델링(dynamic modeling)이 필요할 수 있다.

실내 시스템은 운용 환경을 시설 운영자가 관리하는 경우가 많기 때문에 위치추정 인프라(localization infrastructure)를 설치할 수 있다. 기준 마커, 반사체(reflector), UWB 앵커(anchor), 자기 마커(magnetic marker), QR 형태의 랜드마크, 전용 비콘(beacon)을 이용하면 중요한 위치에서 강력한 기준 정보를 제공할 수 있다. 인프라는 설치 및 유지보수 부담을 증가시키지만 인프라 독립성보다 도킹이나 정밀 상호작용의 반복성이 중요한 장소에서는 매우 높은 위치추정 성능을 제공할 수 있다.

실외 인프라는 넓은 영역에 연속적으로 설치하기가 일반적으로 더 어렵다. 캠퍼스나 산업 현장에서는 특정 위치에 앵커나 측량된 랜드마크(surveyed landmark)를 설치할 수 있지만 장거리 경로 전체가 전용 인프라에 의존하기는 어렵다. 따라서 실외 위치추정은 GNSS와 같은 전역 기준 및 자연 특징 기반 지도 위치추정(natural-feature map localization)을 활용하는 것이 유리하다. 전용 인프라는 도킹 스테이션, 게이트, 적재 구역, GNSS 음영 구간과 같은 핵심 영역에 집중적으로 배치할 수 있다.

지도 요구사항(map requirement)도 서로 다르다. 실내 지도는 일반적으로 제한된 공간을 포함하며 비교적 높은 기하학적 상세도를 유지할 수 있다. 랙 이동, 임시 팔레트, 문, 공사와 같은 변화에 대한 지도 관리가 필요하지만 전체 지도 규모는 관리 가능한 수준이다. 실외 지도는 훨씬 넓은 영역을 포함할 수 있으며 고도(elevation), 도로 형상, 지형, 건물, 식생 경계, 의미론적 랜드마크(semantic landmark), 여러 좌표계를 포함할 수 있다.

전역 좌표 참조(global coordinate referencing)는 로봇 지도가 지리 정보(geographic information), 측량된 기반 시설, 플릿 경로(fleet route), GNSS 좌표와 연결되어야 하는 경우가 많기 때문에 실외에서 더욱 중요하다. 위도-경도-고도(latitude-longitude-altitude), 지구중심 좌표(Earth-centered coordinate), 지역 동-북-상 좌표(local ENU frame)와 같은 좌표계가 지도 및 오도메트리 좌표계와 상호작용할 수 있다. 실내 시스템은 시설별 직교 좌표계(Cartesian frame)만으로 운용할 수 있지만 다중 건물 또는 캠퍼스 수준으로 확장되면 계층적 좌표계 관리(hierarchical frame management)가 필요하다.

수직 위치(vertical position)는 알려진 평면 바닥에서 운용되는 일반적인 실내 평면형 자율이동로봇(planar AMR)에서는 상대적으로 중요도가 낮을 수 있다. 그러나 엘리베이터, 경사로, 다층 건물에서는 층 식별(floor identification)과 수직 전환을 명시적으로 처리해야 한다. 실외 플랫폼은 경사면, 불규칙 지형, 교량, 램프, 고도 변화를 지속적으로 경험한다. 따라서 지형 복잡성과 차량 동역학이 증가할수록 완전한 3차원 자세 추정(3D pose estimation)의 중요성이 증가한다.

지도 관측 가능성(map observability)은 환경의 기하학적 구조에 따라 변화한다. 실내 복도에서는 횡방향 위치를 강하게 제약할 수 있지만 복도 진행 방향의 위치 정보는 상대적으로 약할 수 있다. 넓은 실외 도로에서도 도로 경계를 통해 횡방향 위치는 강하게 제약되지만 특징적인 랜드마크가 없으면 종방향 정보가 약할 수 있다. 따라서 위치추정 성능은 하나의 위치 오차 값만으로 표현하기보다 방향별 특성(directional characteristics)을 고려하여 평가해야 한다.

환경의 동적 변화(environmental dynamics)는 실내와 실외 모두에서 중요하지만 서로 다른 형태로 나타난다. 실내 창고에는 작업자, 지게차, 팔레트, 카트, 변경되는 재고가 존재할 수 있다. 실외 시스템은 차량, 보행자, 움직이는 식생, 건설 작업, 날씨, 계절 변화를 경험한다. 위치추정 알고리즘은 일시적인 객체가 지도 정합(map matching)을 지배하지 않도록 해야 하며 지속적으로 존재하는 기하학적 또는 의미론적 특징을 우선적으로 활용해야 한다.

날씨(weather)는 주로 실외 위치추정에서 중요한 요구사항이다. 비, 눈, 안개, 먼지, 센서 창의 수분, 온도 변화, 직사광선은 여러 센서에 동시에 영향을 줄 수 있다. 눈은 지면과 랜드마크의 가시적인 형상을 변화시킬 수 있으며 비나 먼지는 라이다 반사 신호에 영향을 줄 수 있다. 따라서 실외 위치추정에는 제어된 실내 환경에서는 상대적으로 중요도가 낮을 수 있는 환경 강건성(environmental robustness)이 요구된다.

위치추정에서는 최고 정확도(peak accuracy)보다 가용성(localization availability)이 더 중요할 수 있다. 이상적인 조건에서만 센티미터 수준의 정확도를 제공하는 시스템보다 약간 낮은 정확도라도 지속적으로 유지하는 시스템이 실제 운용에서는 더 유용할 수 있다. 실내 시스템은 인프라와 관리된 지도를 이용하여 센싱 조건을 어느 정도 보장할 수 있다. 반면 실외 시스템은 GNSS, 비전, 라이다 특징, 휠 접지력의 가용성이 경로를 따라 독립적으로 변화할 수 있으므로 더 높은 수준의 중복성(redundancy)이 필요하다.

센서 중복성(sensor redundancy)은 단순히 센서 수를 증가시키는 것이 아니라 고장 다양성(failure diversity)을 기준으로 설계해야 한다. GNSS와 지도 기반 라이다는 서로 다른 원인으로 실패하며 비전과 휠 오도메트리 역시 서로 다른 고장 특성을 가진다. 상호 보완적인 고장 메커니즘을 가진 센서를 결합하면 하나의 정보원이 불안정해질 때 다른 정보원이 위치추정을 지원할 수 있다. 동일한 환경 조건의 영향을 받는 두 센서는 개수만으로 예상하는 것보다 낮은 중복성을 제공할 수 있다.

실내 위치추정 아키텍처는 일반적으로 휠 오도메트리와 IMU를 연속적인 국부 운동 추정에 사용하고 2D 라이다 또는 시각 지도 위치추정을 이용하여 전역 보정을 수행할 수 있다. 중요한 영역에는 UWB나 랜드마크를 추가할 수 있다. 환경이 제한되어 있고 예상 운용 조건이 상대적으로 명확하기 때문에 이러한 구조는 계산 효율성이 높고 높은 반복성을 제공할 수 있다.

실외 자율이동로봇(Outdoor AMR)은 일반적으로 보다 폭넓은 다중 센서 아키텍처(multi-sensor architecture)를 사용하는 것이 유리하다. 휠 오도메트리와 IMU는 높은 주기의 연속성을 제공하고, RTK-GNSS는 사용 가능한 경우 전역 지리 위치(global geographic position)를 제공하며, 3D 라이다 또는 비전은 위성 정보의 신뢰성이 저하될 때 지도 기준 위치추정을 제공한다. EKF, ESKF, 팩터 그래프(factor graph) 또는 관련 추정기를 통한 센서 융합(sensor fusion)은 센서 신뢰도의 변화와 불확실성을 추적하면서 공통 상태(common state)를 유지한다.

전환 관리(transition management)는 실외 위치추정의 핵심적인 요구사항이다. 로봇은 정확한 RTK-GNSS를 사용할 수 있는 개방된 하늘 아래에서 출발하여 건물에 접근하면서 다중경로가 증가하고, 지붕이 있는 적재 구역으로 진입하고, 터널을 통과한 후 다시 실외로 나올 수 있다. 위치추정은 서로 관련 없는 추정값 사이를 갑작스럽게 전환하기보다 점진적이고 안정적으로 성능이 변화해야 한다. 이러한 전환 과정에서는 일관된 좌표계, 공분산 관리(covariance management), 센서 간 교차 검증(cross-sensor validation)이 필수적이다.

실내 시스템 역시 특히 엘리베이터, 층, 방 또는 개별적으로 작성된 지도 영역 사이에서 전환을 경험한다. 이 경우 주요 문제는 위성 신호 가용성보다 이산적인 지도 관리(discrete map management)에 있다. 위치추정 시스템은 올바른 층이나 영역을 식별하고 적절한 지도를 로드하거나 활성화하며 전환 과정에서 자세 관계를 유지하고 시설 내 서로 다른 위치에 존재하는 기하학적으로 유사한 영역을 혼동하지 않아야 한다.

실패 복구(failure recovery)의 요구사항도 공간 규모에 따라 달라진다. 실내 전역 재위치추정(global relocalization)은 제한된 지도에서 라이다 또는 시각 관측을 이용하여 탐색할 수 있으며 인프라 랜드마크를 이용하면 복구를 가속할 수 있다. 실외 전역 복구는 훨씬 넓은 영역을 탐색해야 할 수 있다. GNSS를 사용할 수 있다면 가설 공간(hypothesis space)을 크게 줄일 수 있으며, 위성 위치측정을 사용할 수 없는 경우에는 의미론적 랜드마크, 도로 위상(road topology), 사전 경로 정보, 계층적 지도(hierarchical map)를 이용하여 복구 범위를 제한할 수 있다.

정확도 지표(accuracy metric)는 이러한 영역별 차이를 반영해야 한다. 실내 평가에서는 제한된 공간에서의 횡방향 오차(lateral error), 도킹 반복성(docking repeatability), 헤딩 오차, 지도 기준 절대 궤적 오차(map-relative ATE)를 중요하게 평가할 수 있다. 실외 평가에서는 장거리 드리프트(long-distance drift), GNSS 음영 구간 성능, 전환 동작, 지형별 상대 궤적 오차(RTE), 수직 오차, 복구 시간, 장시간 임무에서의 위치추정 가용성을 추가적으로 평가해야 한다. 어느 환경에서도 평균 오차만으로는 충분하지 않다.

안전 동작(safety behavior)은 위치추정 신뢰도와 연결되어야 한다. 실내에서는 사람, 랙 또는 좁은 통로 주변에서 위치추정을 상실하면 즉각적인 감속이나 정지가 필요할 수 있다. 실외에서는 차량 속도와 지형 조건으로 인해 정지 자체에도 더 긴 거리가 필요할 수 있다. 따라서 내비게이션 시스템은 모든 자세 추정값이 동일한 신뢰도를 가진다고 가정하지 말고 불확실성과 위치추정 건전성(localization health) 정보를 직접 활용해야 한다.

실내 및 실외 위치추정을 완전히 별개의 기술로 취급해서는 안 된다. 기본적인 확률적 추정 원리(probabilistic estimation principle)는 동일하기 때문이다. 두 환경 모두 운동 예측(motion prediction), 관측 갱신(observation update), 보정(calibration), 동기화(synchronization), 불확실성 관리, 고장 검출, 복구가 필요하다. 핵심적인 차이는 어떤 관측 정보를 사용할 수 있는지, 그 품질이 얼마나 빠르게 변화하는지, 운용 연속성을 유지하기 위해 어느 정도의 중복성이 필요한지에 있다.

실내와 실외를 모두 운용하도록 설계된 로봇은 계층형 위치추정 아키텍처(layered localization architecture)를 사용하는 것이 적합하다. 연속적인 국부 추정기(local estimator)는 전역 기준과 독립적으로 부드러운 운동 상태를 유지하고, 여러 전역 위치추정 정보원(global localization source)은 장기적인 드리프트를 제한할 수 있다. 센서 건전성 모니터링(sensor-health monitoring)은 측정값에 대한 신뢰도를 조정하고, 감독 계층(supervisory layer)은 로봇이 실내와 실외 사이를 이동할 때 전환, 재위치추정, 안전한 성능 저하(safe degradation)를 관리할 수 있다.

궁극적으로 실내 위치추정은 인프라와 상세 지도를 비교적 통제할 수 있는 제한적이고 구조화된 환경에서 정밀하고 반복 가능한 내비게이션을 우선시한다. 실외 위치추정은 더 큰 공간 규모, 변화하는 지형, 날씨, 높은 동역학, 간헐적인 전역 기준을 견딜 수 있어야 한다. 신뢰성 높은 자율 시스템은 상호 보완적인 센서, 명시적인 불확실성 추정(explicit uncertainty estimation), 강건한 지도 관리(robust map management), 변화하는 위치추정 조건으로부터의 안정적인 복구(graceful recovery)를 결합함으로써 두 환경의 요구사항을 모두 충족할 수 있다.

## 01.10. Platform Specific Localization AMR Quadruped UAV

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

위치추정 요구사항(localization requirements)은 로봇 플랫폼(robot platform)에 따라 크게 달라진다. 각각의 이동 메커니즘(mobility mechanism)이 서로 다른 운동 제약(motion constraint), 교란(disturbance), 센싱 시점(sensing viewpoint), 고장 형태(failure mode)를 발생시키기 때문이다. 자율이동로봇(Autonomous Mobile Robot, AMR), 4족 보행 로봇(quadruped), 무인항공기(Unmanned Aerial Vehicle, UAV)는 유사한 센서와 추정 알고리즘을 사용할 수 있지만 상태 모델(state model)을 동일하게 취급할 수는 없다. 플랫폼별 위치추정(platform-specific localization)은 추정 과정의 가정을 로봇의 실제 물리적 동역학(physical dynamics)에 맞추는 것에서 시작한다.

자율이동로봇(AMR)은 일반적으로 지면과 지속적으로 접촉한 상태에서 이동한다. 기존의 바퀴형 플랫폼(wheeled platform)은 대체로 평면 운동(planar motion)에 제약되므로 주요 위치추정 상태는 수평 위치 x와 y, 그리고 헤딩(heading) θ를 중심으로 구성되는 경우가 많다. 실외 지형에서는 롤(roll), 피치(pitch), 수직 변위(vertical displacement)도 중요할 수 있지만, 실내 AMR은 평탄한 바닥에서 발생하는 강한 운동 제약을 활용할 수 있다.

휠 오도메트리(wheel odometry)는 엔코더 측정값(encoder measurement)이 바퀴 회전에 대한 직접적인 정보를 제공하고 이를 통해 차량의 변위를 근사할 수 있기 때문에 AMR에서 특히 유용하다. 차동 구동(differential drive), 스키드 스티어(skid-steer), 애커먼 조향(Ackermann steering), 전방향 구동(omnidirectional drive) 플랫폼에는 각각 서로 다른 운동학 모델(kinematic model)이 필요하다. 바퀴와 지면 사이의 접촉 상태가 예측 가능하게 유지되면 오도메트리는 IMU, 라이다(LiDAR), 비전(vision), 전역 위치 측정값과 융합할 수 있는 부드러운 고주기 상대 운동 정보를 제공한다.

휠 오도메트리의 주요 약점은 바퀴 회전량과 실제 지면 이동량 사이에 예측 가능한 관계가 존재한다고 가정한다는 것이다. 슬립(slip), 타이어 변형(tire deformation), 불규칙 지형, 조향 백래시(steering backlash), 페이로드(payload) 변화, 급격한 가속은 이러한 가정을 위반한다. 특히 스키드 스티어 차량은 회전 과정에서 상당한 횡방향 슬립(lateral slip)을 경험할 수 있다. 따라서 실외 AMR에서는 지형이나 기동 조건으로 인해 엔코더 기반 변위의 신뢰성이 저하될 때 운동 불확실성(motion uncertainty)을 동적으로 증가시킬 필요가 있다.

AMR은 센서 높이와 방향이 일반적으로 안정적으로 유지되기 때문에 지도 기준 센싱(map-relative sensing)을 효과적으로 활용할 수 있다. 2차원 라이다(2D LiDAR)는 구조화된 실내 환경에서 효과적인 위치추정을 제공할 수 있으며, 실외 플랫폼에서는 3차원 라이다(3D LiDAR), 카메라, GNSS를 함께 활용하는 것이 유리하다. 안정적인 센서 장착은 반복되는 기하학적 관측을 지도와 비교하기 쉽게 하지만 거친 지형을 주행하는 경우에는 서스펜션 운동(suspension movement)과 섀시 자세(chassis attitude)를 고려해야 한다.

실내 AMR은 제한 없는 6자유도 추정(six-degree-of-freedom estimation)보다 정밀한 횡방향 위치 결정과 반복 가능한 도킹(docking)을 요구하는 경우가 많다. 따라서 위치추정 아키텍처는 휠 오도메트리와 IMU를 연속적인 운동 추정에 사용하고 라이다 지도 정합(LiDAR map matching)을 이용하여 전역 보정(global correction)을 수행하도록 구성할 수 있다. 센티미터 수준의 반복성이 운용상 중요한 위치에서는 기준 마커(fiducial marker), 반사체(reflector), 초광대역(UWB), 도킹 센서(docking sensor)를 추가적인 지역 기준으로 활용할 수 있다.

실외 AMR은 지형에 의해 롤, 피치, 수직 운동, 서스펜션 동역학(suspension dynamics), 더 큰 휠 슬립이 발생하므로 보다 폭넓은 상태 표현(state representation)이 필요하다. 개방된 하늘에서는 실시간 이동측위 위성항법(RTK-GNSS)이 전역 위치를 제약할 수 있으며, 위성 정보의 신뢰성이 저하되면 IMU, 휠 오도메트리, 라이다, 비전이 연속성을 유지한다. 계층형 추정기(layered estimator)는 부드러운 국부 오도메트리를 유지하면서 지리 좌표계 또는 지도 좌표계를 기준으로 장기적인 전역 드리프트를 독립적으로 보정할 수 있다.

4족 보행 로봇(quadruped robot)은 연속적으로 회전하는 바퀴가 아니라 간헐적인 발 접촉(intermittent foot contact)을 통해 이동하기 때문에 근본적으로 다른 위치추정 문제를 가진다. 다리가 지지 단계(stance phase)와 스윙 단계(swing phase)를 번갈아 수행하면서 로봇 본체에는 주기적인 병진, 회전, 수직 진동, 충격이 발생한다. 위치추정 시스템은 변화하는 접촉 상태와 빠르게 변화하는 동역학을 고려하면서 로봇 본체(body)의 운동을 추정해야 한다.

레그 오도메트리(leg odometry)는 관절 엔코더 측정값(joint encoder measurement), 로봇 운동학(robot kinematics), 그리고 어떤 발이 지면에 대해 정지해 있는지에 대한 가정을 결합하여 운동 정보를 제공한다. 안정적인 지지 단계에서는 접촉하고 있는 발이 일시적인 기준점(temporary reference point)의 역할을 할 수 있다. 하나 이상의 접촉 발을 기준으로 본체 운동을 추적함으로써 추정기는 바퀴형 오도메트리와 유사하지만 관절형 다리 운동학(articulated leg kinematics)에 기반한 단기 변위 정보를 얻을 수 있다.

레그 오도메트리의 신뢰성은 접촉 추정(contact estimation)의 정확도에 크게 의존한다. 정지 상태라고 가정한 발도 자갈, 진흙, 얼음, 젖은 표면, 느슨한 토양, 경사 지형에서는 실제로 미끄러질 수 있다. 잘못된 접촉 분류(contact classification)는 본체 운동 추정에 직접적인 오차를 발생시킨다. 따라서 힘 센서(force sensor), 관절 토크 추정(joint torque estimation), 관성 측정, 접촉 확률 모델(contact probability model), 지형 정보를 이용하여 각각의 발이 상태 추정기를 제약하는 데 적합한지를 판단할 수 있다.

4족 보행 로봇의 위치추정에는 일반적으로 완전한 6자유도 자세 추정(full six-degree-of-freedom pose estimation)이 필요하다. 롤, 피치, 요(yaw), 수직 운동이 정상적인 보행 과정 자체에 포함되기 때문이다. 계단을 오르거나 장애물을 넘고, 경사면을 이동하거나 서로 다른 높이에 발을 디딜 때 본체는 지속적으로 기울어질 수 있다. 따라서 많은 실내 AMR에 적합한 평면 위치추정 모델(planar localization model)은 일반적인 4족 보행 로봇 운용에는 충분하지 않다.

IMU 정보는 동적인 운동 과정에서 높은 주기의 본체 각속도(angular velocity)와 비력(specific force)을 측정하기 때문에 4족 보행 로봇 상태 추정에서 핵심적인 역할을 한다. 관절 엔코더와 접촉 제약(contact constraint)은 관성 드리프트(inertial drift)를 제한할 수 있으며, 라이다 또는 시각-관성 오도메트리(visual-inertial odometry)는 환경을 기준으로 한 운동 정보를 제공한다. 추정기는 보행 전환(gait transition)과 발-지면 상호작용으로 발생하는 충격 진동과 빠른 자세 변화에도 강건해야 한다.

4족 보행 로봇에 장착된 라이다와 카메라는 일반적인 AMR에 장착된 센서보다 더 큰 시점 운동(viewpoint motion)을 경험한다. 보행은 주기적인 본체 진동을 발생시키며 계단이나 거친 지형에서는 센서 방향과 높이가 지속적으로 변화한다. 따라서 특히 스캐닝 라이다(scanning LiDAR)에서는 모션 왜곡(motion distortion)이 중요한 문제가 될 수 있다. 정확한 타임스탬프(timestamp), IMU 기반 디스큐잉(IMU-based deskewing), 정밀한 외부 파라미터 보정(extrinsic calibration), 고주기 자세 추정을 통해 기하학적 일관성을 유지할 수 있다.

4족 보행 로봇은 바퀴형 로봇에서는 활용하기 어려운 추가적인 가능성도 제공한다. 본체 자세를 의도적으로 변경하거나 더 좋은 관측 시점을 확보하기 위해 높은 위치로 이동하고, 위치추정이 기하학적으로 퇴화(degenerate)될 때 자세나 위치를 변경할 수 있다. 능동 인지(active perception)는 이러한 이동 능력을 활용하여 구별 가능한 환경 특징이 더 잘 관측되도록 본체 자세 또는 보행 동작을 선택할 수 있다. 따라서 위치추정과 이동 계획(locomotion planning)은 센싱을 완전히 수동적인 과정으로 취급하는 대신 불확실성을 감소시키기 위해 상호 협력할 수 있다.

무인항공기(Unmanned Aerial Vehicle, UAV)는 지속적인 지면 접촉 없이 3차원 공간에서 자유롭게 이동하기 때문에 또 다른 형태의 위치추정 체계를 요구한다. UAV는 세 개의 축을 따라 병진 운동하고 롤, 피치, 요 방향으로 회전할 수 있으므로 연속적인 6자유도 상태 추정이 필요하다. 차량의 가속도와 추력 방향(thrust direction)이 자세 추정에 직접적으로 의존하기 때문에 작은 자세 오차도 빠르게 위치 오차로 확대될 수 있다.

IMU는 대부분의 UAV 상태 추정기(state estimator)에서 고주기 추정의 핵심을 담당한다. 자이로스코프(gyroscope)는 각속도 정보를 제공하고 가속도계(accelerometer)는 비력을 측정하여 자세, 속도, 위치를 빠르게 전파할 수 있도록 한다. 그러나 관성 적분은 바이어스와 잡음을 빠르게 누적한다. 따라서 UAV 위치추정에는 GNSS, 카메라, 라이다, 기압계(barometer), 거리 센서(range sensor) 또는 기타 외부 관측값을 이용한 빈번한 보정이 필요하다.

개방된 실외 환경에서는 GNSS가 UAV 내비게이션의 중요한 전역 기준을 제공한다. RTK-GNSS는 신호 조건이 양호한 경우 높은 정확도가 필요한 응용을 지원할 수 있다. 그러나 건물 주변, 교량 아래, 식생 아래, 산업 구조물 내부 또는 실내 비행에서는 위성 위치측정 성능이 저하되거나 완전히 사용할 수 없게 될 수 있다. 따라서 강건한 자율 운용을 목표로 하는 UAV는 GNSS를 무조건적인 필수 요소로 가정하기보다 GNSS 음영 위치추정(GNSS-denied localization)을 지원해야 한다.

시각-관성 오도메트리(Visual-Inertial Odometry, VIO)는 카메라가 가볍고 IMU와 결합했을 때 풍부한 운동 제약을 제공하기 때문에 UAV에서 특히 중요하다. 특징 기반(feature-based) 또는 직접 방식(direct method)의 시각 알고리즘은 높은 주기로 상대 운동을 추정할 수 있으며 관성 측정값은 단기적인 관측 가능성(observability)을 향상시킨다. 그러나 어두운 환경, 텍스처가 부족한 영역, 눈부심, 빠른 운동, 반복적인 장면 또는 심한 영상 블러(image blur)에서는 VIO 성능이 저하될 수 있다.

라이다-관성 위치추정(LiDAR-inertial localization)은 기하학적 구조가 존재하는 환경에서 운용되는 UAV에 상호 보완적인 장점을 제공한다. 라이다는 가시적인 텍스처에 의존하지 않고 신뢰할 수 있는 깊이 정보를 제공하며 저조도 환경에서도 내비게이션을 지원할 수 있다. 그러나 페이로드 질량, 소비 전력, 센싱 거리, 진동, 모션 왜곡을 고려해야 한다. 특히 소형 UAV는 대부분의 지상 로봇보다 센서 질량과 계산 자원에 훨씬 엄격한 제약을 받는다.

고도 추정(altitude estimation)은 UAV 위치추정에서 명시적으로 고려해야 한다. 기압(barometric pressure)은 유용한 상대 고도 정보를 제공할 수 있지만 날씨, 기류, 압력 변화의 영향을 받는다. 하향 거리 센서(downward range sensor)는 주변 표면까지의 높이를 추정할 수 있지만 장거리 또는 특정 재질에서는 실패할 수 있다. GNSS 고도, 시각 기하(visual geometry), 라이다, 관성 정보를 결합하여 변화하는 비행 조건에서도 수직 상태의 관측 가능성을 유지할 수 있다.

UAV의 위치추정 실패는 저속 지상 로봇에서 발생하는 유사한 실패보다 훨씬 빠르게 심각해질 수 있다. 지상 로봇은 위치추정 신뢰도가 감소하면 안전하게 정지할 수 있지만 항공기는 지속적으로 자세를 안정화하고 충돌을 피해야 한다. 따라서 상태 추정 지연(state-estimation latency), 갱신 주기, 고장 검출은 비행 제어(flight control)와 밀접하게 연결된다. 위치추정 성능이 저하되면 차량을 불안정하게 만들지 않으면서 제어된 대체 동작(fallback behavior)을 수행해야 한다.

이러한 차이에도 불구하고 AMR, 4족 보행 로봇, UAV는 공통적인 상태 추정 구조(estimation structure)를 가진다. 각각의 플랫폼에는 고주기 국부 운동 추정(high-rate local motion estimation), 낮은 드리프트의 외부 관측, 불확실성 전파(uncertainty propagation), 보정(calibration), 동기화(synchronization), 이상치 제거(outlier rejection), 고장 검출이 필요하다. 사용되는 센서는 서로 겹칠 수 있지만 휠 제약, 발 접촉, 관성 동역학, 시각 정보, 라이다 기하, GNSS에 부여되는 상대적 중요도는 플랫폼마다 크게 다르다.

운동 모델(motion model)은 플랫폼별 차이를 가장 명확하게 보여주는 요소 중 하나이다. 바퀴형 AMR은 비홀로노믹 제약(nonholonomic constraint) 또는 휠 운동학 제약을 활용할 수 있고, 4족 보행 로봇은 일시적인 발 접촉 제약을 활용할 수 있으며, UAV는 지면 제약 없이 주로 관성 동역학과 공기역학적 제어 입력(aerodynamic control input)에 의존한다. 부적절한 제약을 적용하면 추정기가 높은 정밀도를 가진 것처럼 보일 수 있지만 실제 플랫폼이 해당 가정을 위반할 때 체계적인 비일관성(systematic inconsistency)이 발생할 수 있다.

관측 가능성(observability) 역시 플랫폼과 운용 조건에 따라 달라진다. 평면 AMR은 주변 벽으로부터 강한 헤딩 및 횡방향 제약을 얻을 수 있고, 4족 보행 로봇은 지형 접촉과 환경 기하 구조로부터 수직 위치와 자세 정보를 얻을 수 있다. UAV는 전체 상태를 제약하기 위해 시각 시차(visual parallax), 깊이 구조(depth structure), 중력(gravity), 외부 위치 기준이 필요할 수 있다. 따라서 위치추정 설계에서는 각각의 운동 단계에서 어떤 상태 변수를 관측할 수 있는지를 고려해야 한다.

추정기 선택(estimator selection)은 이러한 물리적 요구사항을 따라야 한다. 확장 칼만 필터(EKF)와 오차 상태 확장 칼만 필터(ESKF)는 특히 관성 센싱이 핵심적인 경우 고주기 센서 융합에 폭넓게 적용할 수 있다. 팩터 그래프(factor graph)는 더 긴 시간 구간에 걸쳐 정보를 최적화하고 지연되거나 비동기적인 제약을 통합할 수 있다. 파티클 필터(particle filter)는 다봉 형태의 전역 위치추정(multimodal global localization)에 여전히 유용하다. 추정기는 상태 차원, 비선형 동역학, 불확실성 구조, 계산 자원에 따라 선택해야 한다.

좌표계 아키텍처(coordinate-frame architecture)는 상태의 복잡도가 서로 다르더라도 플랫폼 전체에서 일관성을 유지해야 한다. 국부 오도메트리 좌표계(local odometry frame)는 연속적인 운동을 제공하고, 지도 또는 월드 좌표계(map or world frame)는 장기적인 전역 일관성을 제공할 수 있다. 센서 좌표계(sensor frame)와 로봇 베이스 또는 본체 좌표계(base or body frame)는 보정된 강체 변환(rigid transformation)을 통해 연결된다. 이러한 분리 구조를 유지하면 국부 제어 추정값에 불필요한 불연속성을 발생시키지 않으면서 전역 보정을 수행할 수 있다.

고장 검출(failure detection) 역시 플랫폼별 위험 요소를 반영해야 한다. AMR은 휠 슬립, 잘못된 지도 정합(false map match), GNSS 성능 저하를 감지해야 한다. 4족 보행 로봇은 추가적으로 발 접촉 및 다리 운동학 일관성(leg-kinematic consistency)을 모니터링해야 한다. UAV는 시각 추적 상실(visual tracking loss), 관성 센서 이상(inertial anomaly), 고도 불일치(altitude inconsistency), 급격한 전역 기준 성능 저하를 감지해야 한다. 따라서 일반적인 위치추정 건전성 지표(localization-health indicator)에 이동 방식별 진단(mobility-specific diagnostic)을 추가해야 한다.

시험(testing)은 각 플랫폼의 특성을 대표하는 교란을 재현해야 한다. AMR은 미끄러운 노면, 급회전, 경사면, 실내와 실외 위치 기준 사이의 전환 조건에서 평가해야 한다. 4족 보행 로봇은 계단, 느슨한 지형, 발 미끄러짐, 보행 패턴 변화(gait change), 본체 충격을 포함하는 시험이 필요하다. UAV 시험에는 빠른 회전, 고도 변화, GNSS 상실, 텍스처가 부족한 장면, 조명 전환, 공격적인 3차원 기동(aggressive 3D maneuver)이 포함되어야 한다.

다중 플랫폼 로봇 시스템(multi-platform robotic system)에서는 공유 공간 이해(shared spatial understanding)라는 추가적인 요구사항이 발생한다. AMR, 4족 보행 로봇, UAV는 서로 크게 다른 높이와 시점에서 동일한 환경을 관측할 수 있다. 이들의 국부 지도(local map)와 센서 특징(sensor signature)은 직접적으로 일치하지 않을 수 있다. 공통 지리 기준(common geographic reference), 공유 랜드마크(shared landmark), 의미론적 지도(semantic map), 플랫폼 간 지도 표현(cross-platform map representation)을 이용하면 협력 위치추정(cooperative localization)과 정보 교환을 지원할 수 있다.

플랫폼 다양성(platform diversity)은 오히려 장점으로 활용될 수도 있다. UAV는 전역적인 상공 관측(overhead observation)을 제공할 수 있고, 4족 보행 로봇은 바퀴형 로봇이 접근하기 어려운 지형을 조사할 수 있으며, AMR은 안정적인 지상 센싱과 지속적인 페이로드 운반 능력을 제공할 수 있다. 협력 위치추정은 이러한 상호 보완적인 시점을 활용하여 특정 로봇의 위치추정 성능이 저하되거나 환경 관측 가능성이 제한될 때 다른 플랫폼이 공간적 제약(spatial constraint)을 제공하도록 할 수 있다.

궁극적으로 플랫폼별 위치추정(platform-specific localization)은 동일한 추정 알고리즘에 서로 다른 센서를 단순히 장착하는 문제가 아니다. AMR은 바퀴와 구조화된 지상 운동으로부터 유용한 제약을 얻고, 4족 보행 로봇은 다리 운동학과 변화하는 발 접촉으로부터 제약을 얻으며, UAV는 고주기 관성 동역학과 3차원 환경 관측을 결합하여 상태를 추정한다. 신뢰성 높은 위치추정은 각각의 플랫폼이 가진 실제 물리적 거동을 중심으로 센싱, 상태 표현, 운동 제약, 불확실성 모델, 고장 복구를 함께 설계할 때 구현할 수 있다.
