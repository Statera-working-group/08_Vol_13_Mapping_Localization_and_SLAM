**Volume 13. Mapping Localization and SLAM**

# Chapter 02. Probabilistic Robotics

## 02.01. Probabilistic Motion Models Velocity Odometry [w/Code]

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

확률적 모션 모델(Probabilistic Motion Model)은 제어 명령(Control Command)이 불확실성(Uncertainty)을 가진 상태에서 실행될 때 로봇의 상태(State)가 어떻게 변화하는지를 설명한다. 명령된 움직임이 하나의 정확한 자세(Pose)를 만든다고 가정하는 대신, 가능한 결과 자세에 대한 확률 분포(Probability Distribution)를 표현한다. 실제 로봇에서는 휠 슬립(Wheel Slip), 액추에이터 오차(Actuator Error), 불균일한 지형(Uneven Terrain), 시간 편차(Timing Variation), 불완전한 기계적 보정(Mechanical Calibration) 등이 발생하기 때문에 이러한 접근이 필수적이다.

이동 로봇(Mobile Robot)의 상태는 일반적으로 평면 자세(Planar Pose) \\(x_t=(x,y,\\theta)\\)로 표현되며, 여기서 \\(x\\)와 \\(y\\)는 위치(Position), \\(\\theta\\)는 방향각(Heading)을 의미한다. 확률적 상태 전이 모델(Probabilistic Transition Model)은 \\(p(x_t \\mid u_t,x_{t-1})\\)를 추정하며, 이는 이전 상태 \\(x_{t-1}\\)에서 제어 입력(Control Input) \\(u_t\\)를 적용한 후 새로운 상태 \\(x_t\\)에 도달할 확률을 나타낸다. 이러한 분포는 확률적 위치추정(Probabilistic Localization)과 동시적 위치추정 및 지도작성(SLAM) 알고리즘의 예측 메커니즘으로 사용된다.

속도 모션 모델(Velocity Motion Model)은 로봇이 명령되거나 측정된 병진 속도(Translational Velocity)와 회전 속도(Rotational Velocity)를 입력받는다고 가정한다. 일반적인 제어 벡터(Control Vector)는 \\(u_t=(v,\\omega)\\)로 표현할 수 있으며, \\(v\\)는 선속도(Linear Velocity), \\(\\omega\\)는 각속도(Angular Velocity)를 의미한다. 시간 간격(Time Interval) \\(\\Delta t\\) 동안 이 값들은 명목 궤적(Nominal Trajectory)을 결정하며, \\(\\omega\\)가 0이 아닌 경우 이상적인 평면 운동은 대략 \\(v/\\omega\\)의 반경을 갖는 원호(Circular Arc)를 따라 이동한다.

이상적인 결정론적 운동(Deterministic Motion)에서는 속도 명령을 적분하여 하나의 예측 자세(Predicted Pose)를 얻는다. 그러나 실제 운동에서는 유효 병진 속도와 각속도가 명령값과 달라지기 때문에 이러한 예측에서 벗어나게 된다. 따라서 확률적 속도 모델(Probabilistic Velocity Model)은 \\(v\\), \\(\\omega\\), 그리고 경우에 따라 추가적인 방향 드리프트(Heading Drift) 성분에 잡음(Noise)을 도입한다. 이러한 잡음 변수를 샘플링(Sampling)하면 하나의 결정론적 종점 대신 가능한 로봇 자세들의 분포를 얻을 수 있다.

운동 불확실성(Motion Uncertainty)은 일반적으로 잡음의 크기를 병진 운동과 회전 운동에 연관시키는 계수를 이용하여 매개변수화(Parameterization)한다. 빠른 병진 운동은 병진 오차뿐 아니라 방향 교란(Heading Disturbance)을 증가시킬 수 있으며, 큰 회전 운동은 회전 오차와 횡방향 변위(Lateral Displacement)를 동시에 발생시킬 수 있다. \\(\\alpha_1,\\alpha_2,\\alpha_3,\\alpha_4\\)와 같은 매개변수가 이러한 관계를 표현하는 데 자주 사용되며, 이를 통해 확률 모델(Stochastic Model)이 특정 로봇 플랫폼의 물리적 거동을 근사할 수 있다.

그 결과로 생성되는 확률 분포는 일반적으로 원형이 아니라 비등방성(Anisotropic)을 갖는다. 전진 주행 중에는 불확실성이 주로 이동 방향을 따라 증가하고, 방향 불확실성(Heading Uncertainty)이 누적되면서 횡방향 위치 분포도 점차 확대된다. 회전 중에는 병진 불확실성과 회전 불확실성이 상호작용하여 곡선형 또는 부채꼴 형태의 자세 분포를 생성할 수 있다. 이러한 기하학적 특성 때문에 불확실성 전파(Uncertainty Propagation)를 고정된 위치 오차 반경만으로 적절하게 표현하기는 어렵다.

오도메트리 모션 모델(Odometry Motion Model)은 이와 다른 관점에서 로봇의 움직임을 표현한다. 원래의 속도 명령을 직접 모델링하는 대신, 두 시간 단계 사이에서 휠 인코더(Wheel Encoder) 또는 다른 오도메트리(Odometry) 정보원이 보고한 상대 자세 변위(Relative Pose Displacement)를 사용한다. 관측된 변위는 일반적으로 초기 회전(Initial Rotation), 병진 이동(Translation), 최종 회전(Final Rotation)으로 분해되며, 이 세 성분은 로봇의 측정된 증분 운동(Incremental Motion)을 간결하게 표현한다.

두 개의 오도메트리 자세(Odometry Pose)가 주어졌을 때 첫 번째 회전은 로봇이 이동 방향을 향해 얼마나 회전했는지를 나타낸다. 병진 성분은 이동 거리(Travel Distance)를 나타내며, 두 번째 회전은 남아 있는 방향 변화를 나타낸다. 확률적 모델은 추정된 운동 잡음(Motion Noise)에 따라 이 세 성분에 변동을 추가한다. 이후 잡음이 포함된 증분 운동을 파티클(Particle) 또는 상태 가설(State Hypothesis)에 적용하여 예측된 로봇 자세를 생성한다.

오도메트리는 실제 수행된 움직임을 직접 반영한다는 점에서 유용하지만, 이를 실제값(Ground Truth)으로 해석해서는 안 된다. 인코더 양자화(Encoder Quantization), 부정확한 휠 직경(Wheel Diameter), 좌우 휠 보정 오차(Wheel Calibration Error), 백래시(Backlash), 타이어 변형(Tire Deformation), 휠 슬립(Wheel Slip), 노면 불규칙성(Surface Irregularity) 등이 모두 오도메트리 오차를 누적시킨다. 체계적 오차(Systematic Error)는 반복 가능한 편향(Bias)을 만들 수 있으며, 슬립과 같은 비체계적 영향(Nonsystematic Effect)은 인코더 측정만으로 예측하기 어려운 급격한 편차를 발생시킬 수 있다.

따라서 속도 모델(Velocity Model)과 오도메트리 모델(Odometry Model)은 서로 연관되어 있지만 서로 다른 운동 정보원을 나타낸다. 속도 모델은 제어 인터페이스(Control Interface)가 의미 있는 선속도와 각속도를 제공할 때 유용하며, 특히 시뮬레이션(Simulation)과 명령 수준 예측(Command-Level Prediction)에 적합하다. 오도메트리 모델은 로봇 제어기(Robot Controller)가 이미 인코더 측정값을 상대 자세로 적분하는 경우에 편리하다. 적절한 모델의 선택은 센싱 아키텍처(Sensing Architecture), 제어기 인터페이스(Controller Interface), 이동 플랫폼의 특성에 따라 결정된다.

차동 구동 로봇(Differential-Drive Robot)은 대표적인 사례이다. 좌우 휠 속도(Left and Right Wheel Velocity)는 전진 및 회전 운동을 결정하지만, 예상된 휠 동작과 실제 휠 동작 사이의 작은 차이도 자세 불확실성을 발생시킨다. 직선 주행에서도 방향 오차가 누적될 수 있으며, 회전 운동에서는 좌우 휠 반경의 차이나 접지력(Traction) 차이에 따른 오차가 더욱 증폭될 수 있다. 스키드 스티어(Skid-Steer) 플랫폼은 회전 과정에서 타이어 또는 트랙과 지면 사이의 슬립이 본질적으로 발생하기 때문에 일반적으로 더 큰 불확실성을 나타낸다.

야외 자율이동로봇(Outdoor AMR)에서는 단순한 평면 모델의 가정을 특히 주의해서 적용해야 한다. 아스팔트(Asphalt), 콘크리트(Concrete), 자갈(Gravel), 토양(Soil), 경사면(Slope), 연석(Curb), 젖은 노면(Wet Surface), 느슨한 지면(Loose Material)은 서로 크게 다른 슬립 특성(Slip Characteristic)을 발생시킬 수 있다. 따라서 실내에서 보정된 모션 모델은 야외에서 불확실성을 과소평가할 가능성이 있다. 운용 환경이 크게 변화하는 경우 지형 의존형 매개변수(Terrain-Dependent Parameter), 적응형 공분산 추정(Adaptive Covariance Estimation), 학습 기반 잔차 모델(Learned Residual Model)을 이용하여 예측 성능을 향상시킬 수 있다.

확률적 모션 모델은 파티클 필터 위치추정(Particle-Filter Localization)에서 핵심적인 역할을 수행한다. 각각의 파티클은 하나의 가능한 로봇 자세를 나타내며, 모션 모델은 최신 제어 또는 오도메트리 정보를 이용하여 파티클을 전파(Propagation)한다. 운동 잡음을 샘플링하면 예상되는 불확실성에 비례하여 파티클 집합이 확산된다. 이후 센서 관측(Sensor Observation)은 측정값을 더 잘 설명하는 가설에 높은 우도(Likelihood)를 부여하며, 이를 통해 분포가 실제 가능성이 높은 자세 주변으로 수렴하게 된다.

확장 칼만 필터(Extended Kalman Filter)와 같은 가우시안 추정기(Gaussian Estimator) 역시 동일한 기본 개념을 사용하지만, 많은 파티클 대신 상태 평균(State Mean)과 공분산(Covariance)을 전파한다. 비선형 모션 함수(Nonlinear Motion Function)가 새로운 평균을 예측하고, 자코비안(Jacobian)과 프로세스 잡음(Process Noise)의 표현이 공분산의 변화를 결정한다. 프로세스 잡음을 과소평가하면 추정기가 지나치게 확신하는 과신(Overconfidence) 상태가 되고, 반대로 지나치게 크게 설정하면 로봇 운동 정보가 제공하는 예측 가치가 감소한다.

따라서 보정(Calibration)은 임의의 잡음 값을 사용하는 것이 아니라 실제 로봇의 거동을 기반으로 수행해야 한다. 반복적인 직선 주행(Straight-Line Run), 회전(Rotation), 곡선 궤적(Curved Trajectory), 복합 운동 시퀀스(Mixed-Motion Sequence)를 정밀한 기준 측정(Reference Measurement)과 비교할 수 있다. 여기서 얻어진 잔차 분포(Residual Distribution)를 이용하면 병진 및 회전 오차가 명령되거나 측정된 움직임에 따라 어떻게 증가하는지를 파악할 수 있다. 실험은 실제 운용에서 예상되는 속도, 페이로드(Payload), 노면, 타이어 상태, 환경 조건을 포함해야 한다.

실제 구현에서는 특수한 수치적 조건(Numerical Condition)도 처리해야 한다. 각속도가 0에 가까워지면 \\(v/\\omega\\)를 직접 사용하는 수식은 물리적으로는 직선 궤적에 가까워짐에도 불구하고 수치적으로 불안정해질 수 있다. 일반적으로 작은 각속도 임계값(Angular-Rate Threshold)을 설정하고 그 이하에서는 직선 운동 근사(Straight-Motion Approximation)를 적용한다. 또한 방향각이 \\(-\\pi\\)와 \\(+\\pi\\) 경계를 통과할 때 일관성을 유지하기 위해 각도 정규화(Angle Normalization)가 중요하다.

시간 해상도(Temporal Resolution) 역시 예측 품질에 영향을 준다. 긴 적분 시간 간격(Integration Interval)은 일정 속도(Constant Velocity) 가정의 현실성을 낮추기 때문에 모델링 오차를 증가시킬 수 있으며, 지나치게 짧은 시간 간격은 계산 부하를 증가시키고 타임스탬프(Timestamp) 불일치를 더욱 민감하게 만든다. 따라서 안정적인 위치추정은 수학적 운동 방정식뿐 아니라 제어 명령, 인코더, 관성측정장치(IMU), 센서의 동기화된 타임스탬프에도 의존한다. 정확한 시간 정렬(Time Alignment)은 모션 모델 인터페이스의 중요한 구성 요소이다.

현대 로봇 시스템은 고전적인 운동학 모델(Classical Kinematic Model)에 관성 센싱(Inertial Sensing)과 데이터 기반 보정(Data-Driven Correction)을 결합하는 방향으로 발전하고 있다. 휠 오도메트리는 높은 주기로 국부 운동(Local Motion)을 제공하고, 관성측정장치(IMU)는 각운동과 단기 가속도 변화를 제한하는 정보를 제공한다. 위성항법시스템(GNSS), 라이다(LiDAR), 비전 위치추정(Visual Localization)은 누적된 드리프트(Drift)를 주기적으로 보정할 수 있다. 학습 기반 모델(Learned Model)을 이용하여 슬립이나 잔차 운동 오차를 추정할 수도 있지만, 후속 상태 추정(State Estimation)을 위해서는 명시적인 확률 표현(Probabilistic Representation)을 유지하는 것이 중요하다.

핵심적인 공학 원칙은 모션 모델이 단순히 로봇에게 무엇을 하도록 명령했는지를 표현하는 것이 아니라, 실제로 로봇이 어떤 움직임을 수행했을 가능성이 있는지를 표현해야 한다는 것이다. 속도 모델과 오도메트리 모델은 물리적인 운동 불확실성을 위치추정 알고리즘이 처리할 수 있는 수학적 상태 전이 확률(Transition Probability)로 변환한다. 모델의 매개변수가 실제 차량, 지형, 시간 특성, 운용 조건을 적절히 반영한다면 위치추정(Localization), 동시적 위치추정 및 지도작성(SLAM), 내비게이션(Navigation), 자율이동로봇(AMR)의 자율 운용을 위한 견고한 예측 기반을 제공할 수 있다.

## 02.02. Probabilistic Sensor Models Beam Likelihood Field [w/Code]

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

확률적 센서 모델(Probabilistic Sensor Model)은 로봇 자세(Robot Pose), 환경 지도(Environmental Map), 센서가 생성하는 측정값(Measurement) 사이의 관계를 설명한다. 라이다(LiDAR)의 거리 측정값을 완벽하게 정확한 값으로 취급하는 대신, 가정된 로봇 상태에서 특정 관측값이 발생할 가능성을 확률적으로 평가한다. 이러한 관계는 일반적으로 \\(p(z_t \\mid x_t,m)\\)로 표현되며, 여기서 \\(z_t\\)는 현재 측정값, \\(x_t\\)는 로봇 자세, \\(m\\)은 지도를 의미한다.

위치추정(Localization)과 동시적 위치추정 및 지도작성(SLAM)에서 센서 모델은 모션 모델(Motion Model)과 근본적으로 다른 역할을 수행한다. 모션 모델이 로봇이 어디로 이동했을 가능성이 있는지를 예측한다면, 센서 모델은 각각의 예측 자세가 실제로 관측된 환경과 얼마나 일치하는지를 평가한다. 따라서 좋은 센서 모델은 센서 데이터와 지도 사이의 기하학적 일치도(Geometric Agreement)를 우도(Likelihood) 값으로 변환하여 파티클(Particle), 상태 추정(State Estimate), 최적화 목적함수(Optimization Objective)를 갱신할 수 있도록 한다.

2차원 라이다(2D LiDAR)와 같은 거리 센서(Range Sensor)는 센서 원점에서 여러 방향으로 주변 표면까지의 거리를 측정한다. 이상적인 지도와 센서를 가정하면 각도 \\(\\theta\\) 방향으로 방출된 빔(Beam)은 가장 가까운 장애물과 교차하고 정확한 거리를 반환한다. 그러나 실제 측정값은 센서 잡음(Sensor Noise), 예상하지 못한 물체, 측정 누락(Missing Return), 반사(Reflection), 가림(Occlusion), 지도 오차(Map Error), 최대 감지 거리(Maximum Sensing Range)의 제한 등으로 인해 이상적인 값에서 벗어난다.

빔 거리 센서 모델(Beam Range Finder Model)은 거리 측정값이 발생하는 여러 원인을 명시적으로 표현한다. 지도에서 예측한 거리와 가까운 측정값은 일반적으로 예상 거리(Expected Range)를 중심으로 하는 가우시안 분포(Gaussian Distribution)를 이용한 적중 성분(Hit Component)으로 모델링된다. 이 성분은 지도에 존재하는 장애물이 정상적으로 관측될 때 발생하는 일반적인 측정 불확실성을 나타낸다. 표준편차(Standard Deviation)는 측정 거리와 예측 거리의 차이가 증가할 때 우도가 얼마나 빠르게 감소하는지를 결정한다.

예상하지 못한 물체(Unexpected Object)는 또 다른 중요한 측정 형태를 만든다. 사람, 차량, 팔레트(Pallet), 상자 또는 임시 장애물이 센서와 지도상의 표면 사이에 존재하면 예측값보다 짧은 거리가 측정될 수 있다. 빔 모델은 이러한 상황을 예상된 장애물보다 앞에서 발생하는 측정값에 확률을 부여하는 단거리 측정 성분(Short-Reading Component)으로 표현할 수 있다. 이를 통해 동적 물체(Dynamic Object)나 지도에 등록되지 않은 물체 때문에 정상적인 로봇 자세가 즉시 잘못된 것으로 판단되는 것을 방지한다.

거리 센서는 최대 감지 거리 부근의 측정값을 생성하기도 한다. 장애물이 검출되지 않거나, 반사 신호가 지나치게 약하거나, 센서가 설정된 최대 거리(Maximum Range)를 반환할 때 이러한 현상이 발생할 수 있다. 최대 거리 성분(Maximum-Range Component)은 이러한 관측값에 명시적인 확률을 부여한다. 또한 설명하기 어려운 측정값을 표현하기 위해 랜덤 측정 성분(Random-Measurement Component)을 사용할 수 있으며, 일반적으로 유효한 센서 거리 전체에 걸쳐 넓거나 균일한 확률을 부여한다.

이러한 성분들은 혼합 모델(Mixture Model)로 결합할 수 있다. 개념적으로 하나의 거리 관측값에 대한 확률은 적중(Hit), 단거리(Short), 최대 거리(Maximum-Range), 랜덤(Random) 성분의 가중 결합(Weighted Combination)으로 표현된다. \\(z_{\\text{hit}}\\), \\(z_{\\text{short}}\\), \\(z_{\\text{max}}\\), \\(z_{\\text{rand}}\\)와 같은 매개변수는 각 성분의 상대적인 기여도를 결정한다. 가중치는 유효한 확률 혼합을 형성해야 하며 실제 센서와 운용 환경에서 수집한 측정 데이터를 기반으로 보정(Calibration)하는 것이 바람직하다.

고전적인 빔 모델(Beam Model)은 가정된 로봇 자세에서 광선 추적(Ray Casting)을 수행하여 각각의 거리 측정값에 대응하는 예상 거리를 계산한다. 선택된 각 라이다 빔에 대해 광선을 점유 지도(Occupancy Map) 내부로 투사하여 점유 셀(Occupied Cell)과 충돌하거나 센서의 최대 거리에 도달할 때까지 추적한다. 이후 예측 거리와 실제 측정 거리를 비교한다. 여러 빔에 대해 이러한 과정을 반복하여 전체 스캔(Scan)에 대응하는 우도를 계산한다.

물리적으로 직관적이라는 장점이 있지만 반복적인 광선 추적은 높은 계산 비용을 발생시킬 수 있다. 파티클 필터 위치추정(Particle-Filter Localization)은 수백 또는 수천 개의 자세 가설을 유지할 수 있으며, 각각의 가설에 대해 많은 라이다 빔을 평가해야 할 수 있다. 모든 빔이 큰 점유 격자(Occupancy Grid)를 반복적으로 탐색하면 센서 평가가 전체 계산량의 상당 부분을 차지한다. 따라서 빔 서브샘플링(Beam Subsampling), 효율적인 광선 추적, 병렬 처리(Parallel Processing), 대체 우도 계산 방식이 실시간 로봇 시스템에서 중요하다.

우도 필드 모델(Likelihood Field Model)은 널리 사용되는 대안적인 방법이다. 각각의 측정값에 대해 명시적으로 광선 추적을 수행하는 대신, 관측된 라이다 빔의 끝점(Endpoint)이 지도상의 장애물과 얼마나 가까운지를 평가한다. 위치추정을 시작하기 전에 점유 지도를 거리 필드(Distance Field)로 변환하며, 각 자유 셀(Free Cell)은 가장 가까운 점유 셀까지의 거리를 저장한다. 이러한 전처리(Preprocessing)를 통해 스캔 평가 과정에서 장애물까지의 거리를 효율적으로 조회할 수 있다.

가정된 로봇 자세에 대해 각각의 거리 측정값은 센서 좌표계(Sensor Coordinate Frame)에서 지도 좌표계(Map Frame)로 변환된다. 변환된 빔 끝점의 위치를 거리 필드에서 찾고 가장 가까운 장애물까지의 거리를 계산한다. 지도상의 장애물과 가까운 끝점에는 높은 우도가 부여되고, 거리가 멀어질수록 점진적으로 낮은 우도가 부여된다. 일반적으로 장애물 거리의 가우시안 형태 함수(Gaussian-Like Function)를 이용하여 기하학적 거리를 측정 확률로 변환한다.

이러한 방식은 반복적인 광선 추적 비용을 크게 줄이며 작은 지도 오차와 자세 오차에도 더 높은 허용성(Tolerance)을 제공한다. 의미 있는 확률을 얻기 위해 스캔 끝점이 점유 격자 셀에 정확하게 위치할 필요가 없다. 대신 장애물 주변에 형성된 우도 필드는 거리가 증가함에 따라 확률이 부드럽게 감소하는 영역을 제공한다. 이러한 특성은 인접한 자세 가설들이 급격한 이진형 패널티(Binary Penalty) 대신 점진적으로 변화하는 가중치를 받을 수 있기 때문에 파티클 필터 위치추정에 특히 유용하다.

그러나 우도 필드 모델은 특정 거리 측정값이 생성된 물리적 과정(Physical Process)을 상대적으로 명확하게 표현하지 못한다. 주로 끝점과 지도 사이의 일치도를 평가하며 빔 경로 전체에서 발생하는 모든 가림 관계(Occlusion Relationship)를 자연스럽게 모델링하지는 않는다. 해당 빔이 앞쪽의 다른 장애물과 먼저 충돌해야 하는 상황에서도 끝점이 특정 장애물과 기하학적으로 가까울 수 있다. 따라서 상세한 빔 시뮬레이션과 비교하면 일부 물리적 충실도(Physical Fidelity)를 희생하는 대신 계산 효율성과 강건성(Robustness)을 확보한다.

전체 스캔에서 계산되는 센서 우도는 일반적으로 로봇 자세와 지도가 주어졌을 때 각각의 빔이 조건부 독립(Conditionally Independent)이라고 가정한다. 이러한 근사에서는 개별 빔의 확률을 곱하여 전체 스캔 우도를 계산할 수 있다. 그러나 매우 작은 확률값을 반복적으로 곱하면 수치적 언더플로(Numerical Underflow)가 발생할 수 있으므로 실제 구현에서는 로그 우도(Log-Likelihood)를 누적하는 방법을 자주 사용한다. 인접한 라이다 빔 사이에는 상관관계가 존재하기 때문에 독립성 가정은 완벽하지 않지만 실시간 위치추정을 위한 실용적인 근사 방법을 제공한다.

모든 빔을 사용하는 것이 항상 바람직한 것은 아니다. 최신 고밀도 라이다(Dense LiDAR)는 한 번의 스캔에서 수백 개 또는 수천 개의 측정값을 생성할 수 있으며, 그중 상당수는 중복된 기하학적 정보를 포함한다. 빔 서브샘플링은 계산량을 감소시키며 강하게 상관된 측정값들을 독립적인 것으로 처리하면서 발생할 수 있는 과도한 확신(Overconfidence)도 줄일 수 있다. 샘플링 전략은 스캔의 좁은 영역만 선택하는 것이 아니라 유용한 각도 범위와 중요한 구조적 정보를 유지해야 한다.

동적 환경(Dynamic Environment)에서는 추가적인 강건성이 필요하다. 사람, 지게차(Forklift), 차량, 문, 이동식 랙(Movable Rack), 식생(Vegetation), 임시 공사 구조물은 위치추정이 정확한 경우에도 정적 지도(Static Map)와 일치하지 않을 수 있다. 강건한 센서 모델은 랜덤 성분, 이상치 제거(Outlier Rejection), 빔 스키핑(Beam Skipping), 제한된 우도 기여(Capped Likelihood Contribution) 등을 이용하여 개별적인 불일치 측정값의 영향을 제한한다. 목적은 관측된 환경 일부가 변화하더라도 안정적인 위치추정을 유지하는 것이다.

빔 스키핑은 많은 파티클이 유사한 지도 구조를 예측하고 있지만 일부 측정 빔이 지도와 지속적으로 일치하지 않을 때 특히 유용하다. 모든 파티클에 이러한 관측값에 대한 패널티를 부여하는 대신, 위치추정 시스템은 동적 물체나 지도에 없는 물체에서 발생했을 가능성이 높은 측정값을 식별하여 갱신 과정에서 제외할 수 있다. 그러나 지나친 빔 제거는 실제로 유용한 정보를 제거하고 잘못된 자세 가설이 유지되도록 만들 수 있으므로 임계값(Threshold)을 신중하게 설정해야 한다.

지도 해상도(Map Resolution)는 센서 모델의 동작에 직접적인 영향을 준다. 거친 점유 격자는 좁은 구조물을 제거하거나 장애물 경계를 양자화(Quantization)할 수 있으며, 지나치게 세밀한 격자는 메모리와 계산 요구량을 증가시킨다. 따라서 거리 필드는 라이다 정확도, 목표 위치추정 정밀도, 환경의 기하학적 구조에 적합한 해상도로 생성해야 한다. 또한 가능하다면 내비게이션에서 사용하는 장애물 팽창(Inflation) 지도와 위치추정에 사용하는 점유 표현(Occupancy Representation)을 구분해야 한다.

센서 외부 보정(Sensor Extrinsic Calibration) 역시 중요하다. 라이다 좌표계와 로봇 베이스 좌표계(Robot Base Frame) 사이의 변환 관계는 모든 빔 끝점이 지도에 나타나는 위치를 결정한다. 작은 병진 또는 각도 보정 오차도 특히 장거리 측정에서 체계적인 스캔-지도 불일치(Scan-to-Map Disagreement)를 발생시킬 수 있다. 로봇이 이동하는 동안 라이다, 오도메트리, 관성측정장치(IMU) 사이의 타임스탬프 오프셋(Timestamp Offset)도 유사한 문제를 발생시키므로 공간 보정(Spatial Calibration)과 시간 보정(Temporal Calibration)은 센서 모델의 품질과 분리해서 생각하기 어렵다.

야외 자율이동로봇(Outdoor AMR)의 센서 모델은 장거리 측정, 희박한 환경 구조(Sparse Structure), 변화하는 기상 조건, 반사 표면, 식생, 먼지, 이동 차량 등을 처리할 수 있어야 한다. 구조화된 실내 복도에서 보정된 우도 모델은 개방된 야드(Open Yard)나 산업 현장에서 제대로 동작하지 않을 수 있다. 다중 센서 위치추정(Multi-Sensor Localization)은 라이다 우도와 위성항법시스템(GNSS), 관성측정장치(IMU), 휠 오도메트리(Wheel Odometry), 비전(Vision), 레이더(Radar)를 결합하여 모든 환경 조건에서 단일 센싱 방식에만 의존하지 않도록 할 수 있다.

몬테카를로 위치추정(Monte Carlo Localization)에서 센서 모델은 각 파티클의 예측 결과가 최신 스캔과 얼마나 일치하는지를 가중치(Weight)로 변환한다. 변환된 측정값이 지도상의 장애물과 잘 정렬되는 파티클에는 더 큰 가중치가 부여되고, 일치하지 않는 파티클에는 더 작은 가중치가 부여된다. 이후 정규화(Normalization)와 재샘플링(Resampling)을 통해 계산 자원을 상태 공간(State Space)의 가능성이 높은 영역에 집중시킨다. 따라서 측정 모델의 품질은 수렴(Convergence), 위치 복구(Recovery), 위치추정 실패에 대한 저항성에 직접적인 영향을 준다.

실제 매개변수 조정(Practical Tuning)은 임의의 값을 사용하는 대신 기록된 데이터를 기반으로 수행해야 한다. 라이다 스캔, 기준 자세(Reference Pose), 지도, 동적 물체, 대표적인 환경 조건이 포함된 로그(Log)를 재생하면서 적중 분산(Hit Variance), 혼합 가중치(Mixture Weight), 최대 장애물 거리(Maximum Obstacle Distance), 사용 빔 개수, 제거 임계값(Rejection Threshold) 등을 변화시킬 수 있다. 적절한 매개변수는 불완전한 측정값에 대해 비현실적으로 높은 확신을 생성하지 않으면서 잘못된 자세와 올바른 자세를 명확하게 구분할 수 있어야 한다.

따라서 빔 모델(Beam Model)과 우도 필드 모델(Likelihood Field Model)은 어느 하나가 모든 상황에서 절대적으로 우수한 방법이라기보다 상호보완적인 확률 표현으로 이해해야 한다. 빔 모델은 거리 측정이 생성되는 과정을 물리적으로 해석할 수 있는 표현을 제공하고, 우도 필드는 효율적이고 부드러운 스캔-지도 평가(Scan-to-Map Evaluation)를 제공한다. 각 모델의 가정, 계산 비용, 보정 요구사항, 실패 모드(Failure Mode)를 이해하면 위치추정, 동시적 위치추정 및 지도작성(SLAM), 내비게이션(Navigation), 자율이동로봇(AMR)의 운용을 위한 신뢰성 높은 관측 모델(Observation Model)을 구축할 수 있다.

## 02.03. Bayes Filter Algorithm Derivation and Implementation [w/Code]

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

베이즈 필터(Bayes Filter)는 불확실한 제어 입력(Uncertain Control)과 잡음이 포함된 센서 관측(Noisy Sensor Observation)으로부터 동적 시스템(Dynamic System)의 숨겨진 상태(Hidden State)를 재귀적으로 추정하기 위한 일반적인 수학적 프레임워크(Mathematical Framework)를 제공한다. 이동 로봇(Mobile Robot)에서 숨겨진 상태는 로봇의 위치(Position), 방향(Orientation), 속도(Velocity), 랜드마크(Landmark) 또는 완벽하게 측정할 수 없는 다른 변수들을 의미할 수 있다. 필터는 하나의 정확한 추정값을 결정하는 대신 가능한 상태들에 대한 확률 분포(Probability Distribution)를 유지한다.

시간 \\(t\\)에서 로봇 상태는 \\(x_t\\), 제어 또는 운동 정보는 \\(u_t\\), 센서 관측은 \\(z_t\\)로 표현한다. 베이즈 필터는 현재 상태에 대한 정보를 신념 분포(Belief Distribution) \\(bel(x_t)\\)로 표현한다. 이 신념은 이전의 제어 입력과 관측에 포함된 관련 정보를 요약하며, 이후의 예측(Prediction)과 측정 갱신(Measurement Update)에 사용되는 확률적 상태 표현(Probabilistic State Representation)이 된다.

재귀적 공식(Recursive Formulation)은 마르코프 가정(Markov Assumption)에 의존한다. 현재 상태 \\(x_t\\)는 이전 상태 \\(x_{t-1}\\)가 알려져 있다면 전체 과거 궤적에 직접 의존하는 대신 이전 상태와 최신 제어 입력 \\(u_t\\)에 의존한다고 가정한다. 마찬가지로 현재 관측 \\(z_t\\)는 주로 현재 상태와 환경에 의존한다고 가정한다. 이러한 가정을 통해 순차적 상태 추정(Sequential State Estimation)을 계산 가능한 형태로 구성할 수 있다.

최신 센서 관측을 반영하기 전에 베이즈 필터는 예측 단계(Prediction Step)를 수행한다. 예측된 신념(Predicted Belief)은 일반적으로 \\(\\overline{bel}(x_t)\\)로 표현한다. 이전의 가능한 모든 상태에 대해 이전 신념을 적분하고 확률적 상태 전이 모델(Probabilistic Transition Model) \\(p(x_t\\mid u_t,x_{t-1})\\)을 적용하여 계산한다. 개념적으로 각각의 이전 상태가 제어 입력을 실행한 후 로봇이 도달할 수 있는 상태들에 확률 질량(Probability Mass)을 분배한다.

예측 방정식(Prediction Equation)은 \\(\\overline{bel}(x_t)=\\int p(x_t\\mid u_t,x_{t-1})bel(x_{t-1})dx_{t-1}\\)로 표현할 수 있다. 이산 상태 공간(Discrete State Space)에서는 적분이 합산(Summation)으로 변환된다. 이 식은 불확실성 전파(Uncertainty Propagation)를 나타낸다. 이전 신념이 좁은 영역에 집중되어 있더라도 운동 잡음(Motion Noise)으로 인해 확률이 더 넓은 영역으로 확산될 수 있으며, 상태 전이 모델은 제어 불확실성이 시간에 따라 상태 분포를 어떻게 변화시키는지를 결정한다.

예측 이후에는 측정 갱신 단계(Measurement Update)를 통해 새로운 관측 \\(z_t\\)를 반영한다. 센서 모델(Sensor Model) \\(p(z_t\\mid x_t)\\)은 로봇이 실제로 상태 \\(x_t\\)에 있다고 가정할 때 해당 측정값이 발생할 가능성을 평가한다. 사후 신념(Posterior Belief)은 \\(bel(x_t)=\\eta p(z_t\\mid x_t)\\overline{bel}(x_t)\\)로 계산되며, 여기서 \\(\\eta\\)는 최종 확률 분포의 적분값 또는 합이 1이 되도록 만드는 정규화 상수(Normalization Constant)이다.

이러한 갱신 과정은 베이즈 규칙(Bayes' Rule)을 적용한 것이다. 예측된 신념은 현재 상태에 대한 사전 확률(Prior Probability)의 역할을 하고, 측정 우도(Measurement Likelihood)는 각각의 상태 가설(State Hypothesis)과 센서 관측 사이의 일치 정도를 나타낸다. 두 값을 곱하면 정규화되지 않은 사후 확률(Unnormalized Posterior)을 얻을 수 있다. 운동 측면에서 가능성이 높으면서 센서 관측과도 일치하는 상태는 확률이 증가하고, 관측과 모순되는 가설은 확률이 감소한다.

정규화 상수는 \\(\\eta=1/p(z_t)\\)로 표현할 수 있으며, 여기서 증거(Evidence) \\(p(z_t)\\)는 모든 상태 가설을 고려했을 때 현재 관측이 발생할 전체 확률을 나타낸다. 많은 로봇 구현에서는 증거를 독립적으로 계산할 필요가 없다. 먼저 정규화되지 않은 상태 확률 또는 파티클 가중치(Particle Weight)를 계산한 후, 전체 합으로 각각을 나누어 유효한 정규화 확률 분포(Normalized Probability Distribution)를 얻는다.

따라서 전체 베이즈 필터는 반복되는 예측-보정 주기(Prediction--Correction Cycle)를 형성한다. 운동 정보는 신념을 시간적으로 전파하며 일반적으로 불확실성을 증가시키고, 정보성이 높은 관측은 분포를 제한하여 불확실성을 감소시킨다. 이러한 상호작용은 로봇 상태 추정의 핵심 원리를 설명한다. 운동 정보만 사용할 경우 위치 불확실성이 지속적으로 누적되지만, 환경 관측은 누적된 드리프트(Drift)를 보정할 수 있는 정보를 제공한다.

베이즈 필터의 유도(Derivation)는 사후 확률 \\(p(x_t\\mid z_{1:t},u_{1:t})\\)에서 시작하며, 여기서 \\(z_{1:t}\\)는 현재 시점까지의 모든 관측을, \\(u_{1:t}\\)는 제어 입력의 이력을 의미한다. 베이즈 규칙을 적용하면 현재 측정 우도를 이전 상태 확률과 분리할 수 있다. 이후 마르코프 가정을 적용하면 과거의 모든 정보를 이전 신념 \\(bel(x_{t-1})\\)으로 압축할 수 있으며, 이를 통해 실제 필터에서 사용하는 재귀적 방정식을 얻는다.

이러한 재귀적 구조(Recursive Structure)는 로봇이 수시간, 수일 또는 그 이상 지속적으로 운용되면서 센서 측정값을 계속 수신할 수 있기 때문에 중요하다. 전체 센서 및 제어 이력을 저장하고 매번 다시 처리한다면 시간이 지날수록 계산 비용이 증가한다. 모델의 가정이 성립하는 경우 신념은 관련된 과거 정보를 충분히 표현하는 확률적 요약(Probabilistic Summary)으로 기능하며, 각각의 새로운 갱신은 주로 이전 신념, 현재 제어 입력, 현재 측정값만을 이용하여 수행할 수 있다.

베이즈 필터는 하나의 특정 계산 알고리즘이라기보다 추상적인 알고리즘(Abstract Algorithm)이다. 상태 표현 방식과 확률 분포에 대한 가정에 따라 다양한 상태 추정 알고리즘이 도출된다. 분포가 가우시안(Gaussian)이고 시스템 동역학(System Dynamics)이 선형(Linear)인 경우 칼만 필터(Kalman Filter)로 연결된다. 비선형 모델(Nonlinear Model)에서는 확장 칼만 필터(Extended Kalman Filter) 또는 무향 칼만 필터(Unscented Kalman Filter)를 사용할 수 있으며, 임의의 형태와 다중 모드 분포(Multimodal Distribution)는 파티클 필터(Particle Filter)를 이용하여 표현할 수 있다.

히스토그램 필터(Histogram Filter)는 베이즈 필터를 직접적으로 구현하는 또 다른 방법이다. 상태 공간을 이산적인 셀(Cell)로 나누고 각각의 셀에 로봇이 해당 상태에 존재할 확률을 저장한다. 예측 단계에서는 모션 모델(Motion Model)에 따라 셀 사이에 확률을 재분배하고, 보정 단계에서는 각각의 셀에 센서 우도를 곱한다. 개념적으로 단순하고 여러 상태 가설을 동시에 표현할 수 있지만 상태 차원(State Dimensionality)과 해상도가 증가하면 메모리와 계산량이 급격히 증가한다.

파티클 필터는 유한한 개수의 가중 샘플(Weighted Sample)을 이용하여 신념을 근사한다. 예측 단계에서는 각각의 파티클을 확률적 모션 모델을 통해 전파한다. 보정 단계에서는 센서 우도를 이용하여 각 파티클의 가중치를 계산한다. 이후 재샘플링(Resampling)을 통해 높은 확률을 갖는 가설 주변에 새로운 파티클 집합을 생성한다. 몬테카를로 위치추정(Monte Carlo Localization)은 이러한 베이즈 필터 구현을 로봇 자세 추정에 적용한 대표적인 방법이다.

연속 가우시안 구현(Continuous Gaussian Implementation)에서는 명시적인 확률 격자나 샘플 대신 압축된 통계적 매개변수(Statistical Parameter)를 대상으로 예측과 보정을 수행한다. 평균 벡터(Mean Vector)는 추정된 상태를 나타내고 공분산 행렬(Covariance Matrix)은 불확실성과 변수 사이의 상관관계를 표현한다. 분포가 대략 단봉형(Unimodal)이고 가우시안 형태일 경우 매우 효율적이지만, 전역 위치추정(Global Localization)이나 납치 로봇 문제(Kidnapped-Robot Problem)처럼 여러 개의 서로 다른 자세 가설이 동시에 존재해야 하는 경우에는 한계가 있을 수 있다.

수치적 구현(Numerical Implementation)에서는 매우 작은 확률값을 신중하게 처리해야 한다. 다수의 센서 측정 우도를 곱하면 부동소수점 연산(Floating-Point Arithmetic)이 표현할 수 있는 범위보다 작은 값이 빠르게 생성될 수 있다. 로그 확률(Log-Probability)을 사용하면 곱셈을 덧셈으로 변환하여 수치적 안정성(Numerical Stability)을 향상시킬 수 있다. 파티클 방식에서는 가중치도 정확하게 정규화해야 하며, 거의 모든 파티클이 매우 작은 가중치를 받는 퇴화 상황(Degenerate Situation)에 대응할 수 있는 안정적인 복구 또는 재샘플링 전략이 필요하다.

갱신 주기(Update Frequency)의 선택은 정확도와 계산 부하 모두에 영향을 준다. 높은 주기로 입력되는 모든 인코더(Encoder) 또는 관성측정장치(IMU) 데이터를 개별적으로 처리하는 것이 항상 필요한 것은 아니며, 반대로 갱신 간격이 지나치게 길면 운동 불확실성과 선형화 오차(Linearization Error)가 증가할 수 있다. 실제 시스템에서는 오도메트리(Odometry) 또는 관성 측정을 이용하여 높은 주기로 예측을 수행하고, 라이다(LiDAR), 비전(Vision), 위성항법시스템(GNSS)과 같은 정보성이 높은 관측이 입력될 때 상대적으로 계산 비용이 큰 보정을 수행하는 경우가 많다.

좌표계(Coordinate Frame)와 타임스탬프(Timestamp)는 단순한 소프트웨어 세부사항이 아니라 확률적 구현의 일부이다. 센서 관측은 해당 측정이 획득된 시점의 로봇 상태와 대응해야 한다. 지연된 패킷(Delayed Packet), 동기화되지 않은 시계(Unsynchronized Clock), 잘못된 좌표 변환(Frame Transformation), 보상되지 않은 센서 움직임은 올바른 상태 가설과 센서 관측이 서로 불일치하는 것처럼 보이게 만들 수 있다. 이러한 문제는 실제로 필터의 매개변수 조정 문제로 잘못 해석되는 경우가 많다.

모델의 품질(Model Quality) 역시 중요하다. 모션 모델이 불확실성을 과소평가하면 예측 신념이 지나치게 좁아지고 필터가 올바른 센서 정보를 거부할 수 있다. 센서 모델이 지나치게 높은 확신을 가지면 개별 측정값이 사후 확률을 지배하게 되어 이상치(Outlier)가 발생할 때 상태 추정이 불안정해질 수 있다. 반대로 모델의 불확실성을 지나치게 크게 설정하면 유용한 정보가 약화된다. 따라서 공분산과 확률 매개변수는 실제 시스템에서 측정된 거동을 반영해야 한다.

야외 자율이동로봇(Outdoor AMR)에서는 베이지안 프레임워크(Bayesian Framework)를 이용하여 서로 다른 특성을 가진 센서들을 자연스럽게 결합할 수 있다. 휠 오도메트리(Wheel Odometry)는 연속적인 국부 운동 정보를 제공하고, 관성측정장치(IMU)는 방향과 동역학을 제한하며, 위성항법시스템(GNSS)은 전역 위치 정보를 제공한다. 라이다, 비전 또는 레이더(Radar)는 주변 환경 구조와 로봇의 관계를 제공한다. 각각의 관측은 적절한 우도 모델을 통해 결합되며, 불확실성은 각 센서가 사후 신념에 얼마나 강하게 영향을 미치는지를 결정한다.

센서 고장(Sensor Failure)과 환경 변화(Environmental Change) 역시 확률적으로 처리할 수 있다. 건물 주변에서는 위성항법시스템의 신뢰성이 감소할 수 있고, 비나 먼지가 많은 환경에서는 라이다 반환값이 저하될 수 있으며, 카메라는 어둠이나 강한 빛 반사에 영향을 받을 수 있다. 느슨한 지면에서는 휠 오도메트리의 정확도가 저하된다. 적응형 잡음 모델(Adaptive Noise Model), 측정 게이팅(Measurement Gating), 고장 검출(Fault Detection), 센서별 신뢰도(Modality-Specific Confidence)를 이용하면 신뢰성이 낮은 관측이 상태 추정에 미치는 영향을 줄일 수 있다.

강건한 구현(Robust Implementation)은 통제된 데이터와 실제 운용 조건을 대표하는 데이터셋을 이용하여 검증해야 한다. 추정 궤적(Estimated Trajectory)을 기준 실제값(Ground Truth)과 비교하면서 이노베이션 통계(Innovation Statistics), 공분산 일관성(Covariance Consistency), 파티클 다양성(Particle Diversity), 복구 동작(Recovery Behavior)을 확인할 수 있다. 직선 주행, 회전, 고동적 운동, 센서 누락, 모호한 환경 구조, 동적 장애물, 실내외 전환 등을 포함하여 정상적인 조건 밖에서 실패하는 모델 가정을 확인해야 한다.

베이즈 필터의 핵심 가치는 불확실성을 체계적으로 처리하는 데 있다. 예측은 운동 이후 로봇이 어디에 존재할 가능성이 있는지를 계산하고, 보정은 새로운 관측 증거와 일치하는 가능성을 평가하며, 정규화는 다음 주기에서 사용할 갱신된 신념을 생성한다. 이러한 재귀적 확률 아키텍처(Recursive Probabilistic Architecture)는 위치추정(Localization), 센서 융합(Sensor Fusion), 추적(Tracking), 동시적 위치추정 및 지도작성(SLAM)을 비롯한 다양한 자율 로봇 상태 추정 시스템의 개념적 기반을 형성한다.

## 02.04. Grid Based Localization Histogram Filter [w/Code]

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

격자 기반 위치추정(Grid-Based Localization)은 연속적인 확률 분포(Continuous Probability Distribution) 대신 이산적인 셀(Cell)의 집합을 이용하여 가능한 로봇 상태를 표현한다. 각각의 셀은 상태 공간(State Space)의 특정 영역에 대응하며 로봇이 해당 상태에 존재할 확률을 저장한다. 평면 위치추정(Planar Localization)에 적용할 경우 요구되는 정확도와 계산 자원에 따라 격자는 위치 \\((x,y)\\) 또는 위치와 방향 \\((x,y,\\theta)\\)을 표현할 수 있다.

히스토그램 필터(Histogram Filter)는 베이즈 필터(Bayes Filter)를 이산적으로 구현한 방법이다. 연속 상태에 대한 확률을 적분하는 대신 유한한 개수의 구간(Bin)을 이용하여 신념 분포(Belief Distribution)를 근사한다. 상태 공간이 \\(x\^1,\\ldots,x\^N\\)의 셀로 구성되어 있다면 필터는 합이 1이 되는 확률 \\(bel(x\^i)\\)를 유지한다. 이러한 명시적 표현을 통해 불확실성을 직접 관찰할 수 있으며 서로 경쟁하는 여러 위치 가설(Localization Hypothesis)을 자연스럽게 동시에 유지할 수 있다.

초기화(Initialization)는 재귀적 위치추정(Recursive Localization)을 시작하기 전의 사전 신념(Prior Belief)을 결정한다. 대략적인 초기 자세가 알려져 있다면 해당 영역 주변에 확률을 집중시킬 수 있다. 초기 자세를 전혀 알 수 없는 경우에는 모든 유효한 자유 공간 셀(Free-Space Cell)에 확률을 균일하게 분배할 수 있다. 이러한 특성으로 인해 격자 기반 위치추정은 미리 정의된 하나의 시작 자세 주변에서 단일 가우시안 근사(Gaussian Approximation)를 요구하지 않으므로 전역 위치추정(Global Localization)에 적합하다.

예측 단계(Prediction Stage)는 로봇 모션 모델(Motion Model)에 따라 이산적인 신념을 전파한다. 각각의 목적 셀 \\(x_t\^i\\)에 대한 예측 확률은 가능한 모든 이전 셀로부터 \\(\\overline{bel}(x_t\^i)=\\sum_j p(x_t\^i\\mid u_t,x_{t-1}\^j)bel(x_{t-1}\^j)\\)를 이용하여 계산된다. 상태 전이 확률(Transition Probability)은 제어 입력 \\(u_t\\)를 실행한 이후 로봇이 하나의 격자 상태에서 다른 상태로 이동할 가능성을 나타낸다.

운동 불확실성(Motion Uncertainty)은 확률이 인접한 셀로 확산되도록 만든다. 전진 명령은 주요 확률 질량(Probability Mass)을 이동 방향으로 이동시키면서 예상 도착 지점 주변에도 작은 확률을 분배할 수 있다. 회전 불확실성(Rotational Uncertainty)은 신념을 여러 방향 구간(Orientation Bin)으로 확산시키고 이후 위치 불확실성을 증가시킬 수 있다. 따라서 유용한 측정값 없이 예측을 반복하면 로봇 운동의 누적된 불확실성을 반영하여 분포가 점차 넓어진다.

보정 단계(Correction Stage)는 최신 센서 관측(Sensor Observation)을 반영한다. 각각의 격자 셀에 대해 센서 모델(Sensor Model)은 \\(p(z_t\\mid x_t\^i,m)\\)을 계산하며, 이는 로봇이 지도 \\(m\\)에서 상태 \\(x_t\^i\\)에 있을 때 측정값 \\(z_t\\)가 관측될 가능성을 나타낸다. 사후 확률(Posterior)은 \\(bel(x_t\^i)=\\eta p(z_t\\mid x_t\^i,m)\\overline{bel}(x_t\^i)\\)를 이용하여 계산하며, 이후 모든 셀에 대해 정규화(Normalization)를 수행한다.

센서 측정값은 환경과의 일치 정도에 따라 격자의 확률 분포를 재구성한다. 예측된 라이다(LiDAR), 소나(Sonar), 비전(Vision) 또는 다른 센서 관측이 실제 측정값과 잘 일치하는 셀에는 높은 확률이 부여된다. 관측을 제대로 설명하지 못하는 셀의 확률은 감소한다. 예측과 보정 주기를 반복하면 일반적으로 로봇의 운동 이력(Motion History)과 환경 측정값 모두에 일치하는 상태 주변으로 확률이 집중된다.

히스토그램 필터의 주요 장점 중 하나는 다중 모드 분포(Multimodal Distribution)를 명시적으로 표현할 수 있다는 것이다. 대칭적인 복도, 창고 통로(Warehouse Aisle), 반복적인 산업 환경에서는 서로 멀리 떨어진 여러 자세가 초기에 유사한 센서 관측을 생성할 수 있다. 가우시안 추정기(Gaussian Estimator)는 이러한 분리된 가능성을 표현하기 어렵지만, 히스토그램은 추가적인 운동이나 센싱 정보가 잘못된 가설을 제거할 때까지 서로 다른 여러 확률 피크(Probability Peak)를 유지할 수 있다.

이러한 특성은 전역 위치추정과 위치 복구(Localization Recovery)에 특히 유용하다. 전역 위치추정에서는 로봇이 자신의 위치를 알지 못한 상태에서 시작할 수 있으므로 초기 신념은 넓은 영역에 분산된다. 관측이 누적되면 일치하지 않는 셀의 확률은 감소하고 가능성이 높은 영역만 유지된다. 로봇이 위치를 잃어버린 경우에는 신념을 다시 확장하거나 초기화하여 잘못된 자세 주변에 갇히지 않고 지도 전체에서 위치를 다시 탐색할 수 있다.

가장 큰 한계는 계산 복잡도(Computational Complexity)이다. 높은 공간 해상도(Spatial Resolution)를 갖는 2차원 격자만으로도 수백만 개의 셀을 포함할 수 있으며, 방향을 추가하면 상태 공간에 또 하나의 차원이 추가된다. 예를 들어 방향각을 여러 각도 구간으로 나누면 전체 상태 수는 방향 구간의 개수만큼 증가한다. 해상도를 높이면 표현 정확도는 향상되지만 메모리 사용량과 예측 및 센서 평가의 계산 비용도 함께 증가한다.

이러한 관계는 이산화 절충(Discretization Trade-Off)을 보여준다. 큰 셀은 계산량을 줄이지만 추정된 자세를 양자화(Quantization)하고 중요한 기하학적 차이를 제거할 수 있다. 작은 셀은 더 정밀한 위치추정을 가능하게 하지만 필요한 계산 자원을 크게 증가시킨다. 따라서 적절한 해상도는 단순히 최대의 수치 정밀도를 목표로 선택하는 것이 아니라 로봇 크기, 센서 정확도, 지도 구조, 위치추정 요구사항, 사용 가능한 연산 자원, 예상 운용 영역을 종합적으로 고려하여 결정해야 한다.

방향 이산화(Orientation Discretization)는 별도로 고려해야 할 중요한 요소이다. 위치가 거의 동일하더라도 방향이 다른 두 상태는 크게 다른 거리 센서 측정값을 생성할 수 있다. 각도 구간이 지나치게 크면 예측된 센서 기하 구조가 부정확해져 위치추정 품질이 저하될 수 있다. 반대로 필요 이상으로 세밀하면 성능 향상에 비해 계산량만 증가한다. 따라서 공간 해상도와 각도 해상도는 로봇 및 센서 구성에 맞추어 함께 조정해야 한다.

효율적인 예측은 모션 모델의 구조적 특성을 활용할 수 있다. 모든 셀 쌍 사이의 상태 전이를 계산하는 대신 전이 확률이 의미 있게 존재하는 국부적인 인접 영역(Local Neighborhood)으로 갱신 범위를 제한할 수 있다. 합성곱 형태 연산(Convolution-Like Operation), 희소 전이 커널(Sparse Transition Kernel), 분리 가능한 근사(Separable Approximation), 병렬 계산(Parallel Computation)을 활용하면 계산 비용을 추가로 감소시킬 수 있다. 이러한 최적화는 불가능하거나 확률이 거의 없는 상태 전이를 제거하면서 확률적 해석을 유지한다.

각각의 셀에 대해 가상 센서 관측을 계산해야 하는 경우 측정 갱신(Measurement Update)이 가장 많은 계산 비용을 요구할 수 있다. 사전 계산된 우도 필드(Precomputed Likelihood Field), 거리 변환(Distance Transform), 축소된 빔 집합(Reduced Beam Set), 계층형 지도(Hierarchical Map), 캐시된 기하 정보(Cached Geometric Information)를 이용하면 스캔-지도 평가(Scan-to-Map Evaluation)를 가속할 수 있다. 사전 확률이 매우 낮은 셀을 일시적으로 고비용 계산에서 제외할 수도 있지만 전역 복구에 필요한 가설까지 제거하지 않도록 신중하게 설계해야 한다.

각각의 보정 이후에는 격자 전체 확률의 합이 1이 되도록 정규화를 수행해야 한다. 대규모 격자에서는 많은 셀이 극도로 작은 확률값을 가질 수 있어 수치 정밀도(Numerical Precision) 문제가 발생할 수 있다. 로그 영역 계산(Log-Domain Computation)을 이용하면 우도 계산의 안정성을 향상시킬 수 있으며, 확률 하한값(Probability Floor)을 설정하면 복구 가능성이 있는 상태가 수치적으로 완전히 제거되는 것을 방지할 수 있다. 그러나 인위적인 하한값이 실제 센서 증거를 압도하지 않도록 충분히 작게 설정해야 한다.

지도(Map)와 위치추정 격자(Localization Grid)는 서로 관련되어 있지만 개념적으로는 다른 구조이다. 점유 격자(Occupancy Grid)는 환경 공간을 자유(Free), 점유(Occupied), 미확인(Unknown) 상태 또는 그 확률로 표현한다. 반면 위치추정 격자는 로봇 상태에 대한 확률을 표현한다. 하나의 위치추정 상태는 위치와 방향을 포함할 수 있으며 환경 지도를 이용하여 평가된다. 이 두 격자를 혼동하면 해상도, 메모리 요구량, 센서 모델 계산에 대한 잘못된 가정으로 이어질 수 있다.

격자 기반 위치추정은 지도 품질(Map Quality)의 영향도 크게 받는다. 잘못된 벽 구조, 이동된 랙(Rack), 임시 장애물, 식생(Vegetation), 공사로 인한 변화, 오래된 야외 구조물 정보는 올바른 로봇 자세에서도 측정 우도를 감소시킬 수 있다. 강건한 센서 모델(Robust Sensor Model)은 지도와의 부분적인 불일치를 허용해야 한다. 산업 현장에서는 위치추정 알고리즘 자체의 개선만큼 지도 버전(Map Version)을 관리하고 장기적인 구조 변화를 식별하는 것도 중요하다.

야외 자율이동로봇(Outdoor AMR)에서는 조밀한 3차원 또는 대규모 영역의 히스토그램 표현이 일반적으로 높은 계산 비용을 요구하지만 제한된 문제에서는 격자 개념을 유용하게 활용할 수 있다. 로봇은 여러 위치 가설을 표현하기 위해 낮은 해상도의 전역 격자(Coarse Global Grid)를 사용한 후 가능성이 높은 영역 주변에서 고해상도 국부 추정기(High-Resolution Local Estimator)를 활성화할 수 있다. 위성항법시스템(GNSS)은 초기 탐색 영역을 제한하고 라이다, 비전, 레이더(Radar), 지도 특징(Map Feature)은 확률 분포를 더욱 정밀하게 조정할 수 있다.

계층형 및 다중 해상도 격자(Hierarchical and Multi-Resolution Grid)는 실용적인 확장 방법을 제공한다. 확률이 낮은 넓은 영역은 낮은 해상도로 표현하고, 높은 확률을 가진 영역은 더 작은 셀로 세분화할 수 있다. 이러한 자원 할당 방식은 높은 위치추정 정밀도가 필요한 영역에 계산 자원을 집중시킨다. 유사한 원리는 적응형 공간 구조(Adaptive Spatial Structure), 거친 단계에서 세밀한 단계로 진행하는 위치추정(Coarse-to-Fine Localization), 전역 이산 탐색에서 연속적인 국부 추정으로 전환하는 하이브리드 시스템(Hybrid System)에서도 사용된다.

최대 사후 확률 추정(Maximum A Posteriori Estimate)은 가장 높은 사후 확률을 갖는 셀을 선택하여 얻을 수 있지만 전체 신념 분포는 이러한 단일 결과보다 더 많은 정보를 포함한다. 주요 피크 주변의 확률 질량은 위치추정 신뢰도(Localization Confidence)를 나타내고, 보조 피크(Secondary Peak)는 아직 해결되지 않은 모호성(Ambiguity)을 보여준다. 따라서 응용 시스템에서는 내비게이션 또는 안전 관련 의사결정을 수행할 때 추정된 자세뿐 아니라 전체 확률 분포의 형태도 함께 고려해야 한다.

실제 구현에서는 일반적으로 단순한 재귀적 순서를 반복한다. 이전 격자 신념을 오도메트리(Odometry) 또는 제어 정보를 이용하여 전파하고, 후보 상태에 대한 센서 우도를 평가한 다음, 예측 확률과 해당 우도를 곱하고 결과 격자를 정규화한다. 새로운 운동 및 관측 정보가 입력될 때마다 이러한 주기를 반복하여 로봇의 사후 상태 분포(Posterior State Distribution)에 대한 이산적인 근사를 지속적으로 유지한다.

검증(Validation)에서는 평균 위치 오차만을 평가해서는 안 된다. 초기 위치를 알 수 없는 상태에서의 수렴(Convergence), 대칭적인 환경에서의 동작, 잘못된 관측에 대한 강건성, 의도적으로 로봇 자세를 변경한 이후의 복구, 격자 해상도에 대한 민감도를 실험해야 한다. 또한 이론적으로 정확한 구성이 실제 실시간 시스템에서 사용하기 어려울 수 있으므로 실행 시간(Runtime), 메모리 사용량, 갱신 주기, 확률 집중도(Probability Concentration)도 함께 측정해야 한다.

히스토그램 필터는 베이즈 필터를 가장 명확하게 이해할 수 있는 계산 형태 중 하나로 보여준다. 예측은 확률을 이동시키면서 확산시키고, 측정 갱신은 관측에 의해 지지되는 상태의 확률을 강화하며, 정규화는 유효한 신념 분포를 유지한다. 조밀한 격자는 큰 상태 공간에서 높은 계산 비용을 요구할 수 있지만, 불확실성과 여러 상태 가설을 명시적으로 표현할 수 있다는 특성으로 인해 격자 기반 위치추정은 확률 로보틱스(Probabilistic Robotics)의 중요한 개념적·실용적 기반을 형성한다.

## 02.05. Gaussian Process for Robot Mapping [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

가우시안 프로세스(Gaussian Process)는 알려지지 않은 함수(Unknown Function)를 가능한 함수들의 확률 분포(Probability Distribution)로 표현하는 비모수 베이지안 모델(Nonparametric Bayesian Model)이다. 로봇 매핑(Robot Mapping)에서 알려지지 않은 함수는 점유 상태(Occupancy), 지형 고도(Terrain Elevation), 표면 특성(Surface Property), 주행 가능성(Traversability), 신호 강도(Signal Strength) 또는 다른 공간적 물리량(Spatial Quantity)을 나타낼 수 있다. 가우시안 프로세스는 측정된 값만 저장하는 대신 관측되지 않은 위치의 값과 그 불확실성(Uncertainty)을 함께 예측한다.

가우시안 프로세스는 일반적으로 \\(f(\\mathbf{x})\\sim GP(m(\\mathbf{x}),k(\\mathbf{x},\\mathbf{x\'}))\\)로 표현하며, 여기서 \\(m(\\mathbf{x})\\)는 평균 함수(Mean Function), \\(k(\\mathbf{x},\\mathbf{x\'})\\)는 공분산 함수(Covariance Function) 또는 커널 함수(Kernel Function)이다. 입력 \\(\\mathbf{x}\\)는 일반적으로 공간 좌표(Spatial Coordinate)를 나타내며 \\(f(\\mathbf{x})\\)는 매핑하려는 물리량을 의미한다. 커널은 서로 다른 위치에서 얻어진 관측값들이 서로에게 얼마나 강하게 영향을 미치는지를 결정한다.

평균 함수는 관측값이 반영되기 이전에 예상되는 값을 나타낸다. 많은 응용에서는 공간적 구조가 주로 공분산 함수에 의해 표현되기 때문에 평균 함수를 0 또는 다른 단순한 사전값(Prior)으로 초기화한다. 물리적 지식을 사용할 수 있는 경우에는 보다 정보성이 높은 사전값을 사용할 수도 있다. 예를 들어 예상 지형 고도, 알려진 바닥 구조 또는 기존에 생성된 지도는 의미 있는 초기 평균을 제공할 수 있다.

커널은 공간적 상관관계(Spatial Correlation)에 대한 가정을 정의하기 때문에 가우시안 프로세스 매핑의 핵심 요소이다. 일반적으로 서로 가까운 위치는 멀리 떨어진 위치보다 높은 공분산을 갖지만 정확한 관계는 선택한 커널에 따라 달라진다. 대표적인 커널에는 제곱 지수 커널(Squared Exponential Kernel)과 마테른 커널(Matérn Kernel)이 있다. 이들의 하이퍼파라미터(Hyperparameter)는 특성 길이 척도(Characteristic Length Scale), 신호 분산(Signal Variance), 추정된 공간장의 부드러움(Smoothness) 등을 결정한다.

로봇이 학습 위치(Training Location) \\(X\\)에서 측정값 \\(\\mathbf{y}\\)를 수집한다고 가정할 수 있다. 센서 잡음(Sensor Noise)은 공분산 행렬(Covariance Matrix)에 잡음 분산 \\(\\sigma_n\^2\\)을 추가하여 표현할 수 있다. 질의 위치(Query Location) \\(X_\*\\)에 대해 가우시안 프로세스 회귀(Gaussian Process Regression)는 관측값을 조건으로 하는 사후 예측 분포(Posterior Predictive Distribution)를 계산한다. 결과는 단순한 예측 평균뿐만 아니라 각 질의 위치에서 모델이 얼마나 불확실한지를 나타내는 예측 분산(Predictive Variance)도 포함한다.

예측 평균(Predictive Mean)은 \\(\\boldsymbol{\\mu}_\*=K_{\*X}(K_{XX}+\\sigma_n\^2I)\^{-1}\\mathbf{y}\\)로 표현할 수 있다. 예측 공분산(Predictive Covariance)은 \\(\\Sigma_\*=K_{**}-K_{\*X}(K_{XX}+\\sigma_n\^2I)\^{-1}K_{X\*}\\)로 표현된다. 이러한 방정식은 관측 데이터가 커널 관계를 통해 새로운 위치의 예측에 어떻게 영향을 미치는지 보여주는 동시에, 측정값을 조건으로 반영한 이후에도 남아 있는 불확실성을 정량적으로 표현한다.

이러한 불확실성 표현은 로보틱스(Robotics)에서 가우시안 프로세스가 제공하는 가장 중요한 장점 중 하나이다. 신뢰성 높은 측정값이 조밀하게 존재하는 영역은 일반적으로 낮은 사후 분산(Posterior Variance)을 나타내지만, 탐색되지 않았거나 관측이 부족한 영역은 높은 불확실성을 유지한다. 따라서 로봇은 충분한 측정을 통해 안전하다고 판단된 영역과 단순히 정보가 부족하여 위험 여부를 알 수 없는 영역을 구분할 수 있다. 이러한 차이는 자율 탐색(Autonomous Exploration)과 위험 인지형 계획(Risk-Aware Planning)에 중요하다.

가우시안 프로세스 매핑은 기존의 점유 격자 매핑(Occupancy-Grid Mapping)과 근본적으로 다른 방식으로 공간을 표현한다. 점유 격자는 공간을 독립적이거나 국부적으로 갱신되는 셀(Cell)로 분할하지만, 가우시안 프로세스는 커널을 통해 공간 위치 사이의 상관관계를 모델링한다. 따라서 측정값은 주변의 관측되지 않은 위치에도 연속적으로 영향을 줄 수 있다. 이를 통해 부드러운 확률 지도(Probabilistic Map)를 생성하고 이산화 인공물(Discretization Artifact)을 줄일 수 있지만, 일반적으로 단순한 격자 갱신보다 높은 계산량이 필요하다.

점유 매핑(Occupancy Mapping)에서는 잠재 함수(Latent Function)를 이용하여 특정 공간 위치가 자유 공간(Free Space) 또는 점유 공간(Occupied Space)에 가까운지를 표현할 수 있다. 점유 관측은 연속적인 가우시안 측정값이 아니라 이진 데이터(Binary Data)이기 때문에 일반적인 회귀보다 가우시안 프로세스 분류(Gaussian Process Classification)가 적합한 경우가 많다. 연결 함수(Link Function)는 잠재 GP 값을 점유 확률로 변환하여 공간적 상관관계를 유지하면서 자유 공간과 장애물에 대한 확률적 추정을 가능하게 한다.

거리 측정값(Range Measurement)을 학습 정보로 변환할 때에는 주의가 필요하다. 라이다(LiDAR) 빔은 반환점 이전의 위치가 자유 공간이라는 증거를 제공하고, 빔의 끝점(Endpoint)은 점유된 표면이 존재한다는 증거를 제공한다. 이러한 관측을 학습점(Training Point)으로 샘플링하거나 가우시안 프로세스 점유 매핑(Gaussian Process Occupancy Mapping)에 특화된 방법으로 처리할 수 있다. 샘플링 전략은 계산 비용, 지도 세부 표현, 자유 공간과 점유 공간 증거 사이의 균형에 영향을 준다.

가우시안 프로세스는 지형 및 고도 매핑(Terrain and Elevation Mapping)에 특히 자연스럽게 적용할 수 있다. 입력은 수평 좌표 \\((x,y)\\)로 구성하고 목표 함수 \\(f(x,y)\\)는 표면 높이(Surface Height)를 나타낼 수 있다. 라이다, 스테레오 비전(Stereo Vision), 깊이 카메라(Depth Camera) 또는 다른 거리 센서에서 얻은 관측을 이용하여 연속적인 고도 표면을 추정할 수 있다. 예측 분산은 지형 고도가 충분히 알려지지 않은 영역과 추가적인 센싱이 필요한 영역을 식별할 수 있도록 한다.

동일한 프레임워크를 이용하여 주행 가능성과 관련된 물리량을 추정할 수도 있다. 적절한 관측 데이터를 확보할 수 있다면 지형 거칠기(Terrain Roughness), 경사도(Slope), 순응성(Compliance), 마찰(Friction), 식생 밀도(Vegetation Density), 예상 휠 슬립(Expected Wheel Slip)을 공간 함수로 모델링할 수 있다. 로봇은 이동 경로를 계획할 때 예측값과 불확실성을 함께 고려할 수 있다. 예측 난이도는 중간 정도이지만 불확실성이 낮은 경로가 쉬워 보이지만 특성이 거의 알려지지 않은 경로보다 더 적합할 수 있다.

커널 설계(Kernel Design)는 최종적으로 생성되는 지도에 큰 영향을 미친다. 긴 길이 척도(Long Length Scale)는 부드러운 예측을 생성하지만 작은 환경 구조를 제거할 수 있고, 짧은 길이 척도(Short Length Scale)는 국부적인 변화를 보존하지만 잡음에 민감해지고 더 조밀한 관측을 요구할 수 있다. 등방성 커널(Isotropic Kernel)은 모든 방향에서 유사한 공간적 특성을 가정하지만, 비등방성 커널(Anisotropic Kernel)은 축에 따라 서로 다른 상관 길이를 표현할 수 있다. 따라서 커널 선택은 매핑하려는 물리적 특성을 반영해야 한다.

하이퍼파라미터는 공학적 지식을 바탕으로 수동으로 선택하거나 주변 우도(Marginal Likelihood)를 최대화하여 데이터로부터 학습할 수 있다. 주변 우도는 관측 데이터와의 일치성과 모델 복잡도(Model Complexity) 사이의 균형을 제공하며 커널 매개변수를 체계적으로 조정할 수 있는 방법을 제공한다. 그러나 데이터가 제한적이거나 공간적으로 불균일하게 분포된 경우 최적화가 부적절한 국소해(Local Solution)에 수렴할 수 있으므로 학습된 매개변수도 예상되는 물리적 거동 및 검증 데이터와 비교하여 확인해야 한다.

표준 가우시안 프로세스(Standard Gaussian Process)의 주요 한계는 계산 복잡도의 확장성(Computational Scaling)이다. 정확한 GP 추론(Exact GP Inference)은 공분산 행렬 연산이 데이터 증가에 따라 급격하게 복잡해지기 때문에 \\(N\\)개의 학습 관측값에 대해 일반적으로 약 \\(O(N\^3)\\)의 계산량과 \\(O(N\^2)\\)의 메모리를 요구한다. 로봇 매핑에서는 수천 개에서 수백만 개의 측정값이 생성될 수 있으므로 대규모 환경이나 장시간 임무에서는 직접적인 정확 추론을 적용하기 어렵다.

희소 가우시안 프로세스(Sparse Gaussian Process)는 데이터셋을 더 적은 수의 유도점(Inducing Point)으로 표현하여 이러한 문제를 완화한다. 모든 관측값을 전체 공분산 계산에 동일하게 참여시키는 대신 대표적인 위치들을 이용하여 중요한 공간 정보를 요약한다. 유도점의 개수와 배치는 계산 효율성(Computational Efficiency)과 근사 정확도(Approximation Accuracy) 사이의 절충 관계를 결정한다.

국부 가우시안 프로세스 매핑(Local Gaussian Process Mapping)은 확장성을 확보하기 위한 또 다른 방법이다. 환경을 여러 공간 영역으로 나누고 각 영역에서 독립적이거나 부분적으로 중첩되는 GP 모델을 유지한다. 로봇은 전체 전역 데이터셋을 처리하는 대신 현재 위치 주변과 관련된 모델만 평가한다. 이러한 방식은 메모리 증가를 제한하면서 온라인 매핑(Online Mapping)을 지원할 수 있지만 국부 모델 사이의 경계와 일관성(Consistency)을 신중하게 관리해야 한다.

자율 로봇은 지속적으로 새로운 관측값을 획득하기 때문에 증분 매핑(Incremental Mapping)이 중요하다. 새로운 측정값이 들어올 때마다 전체 가우시안 프로세스를 처음부터 다시 계산하는 것은 비효율적이다. 온라인 근사(Online Approximation), 슬라이딩 국부 데이터셋(Sliding Local Dataset), 희소 갱신(Sparse Update), 정보성이 높은 샘플의 능동적 선택(Active Selection)을 이용하면 계산 부담을 줄일 수 있다. 중복된 측정값은 제거하거나 압축하고 불확실성을 크게 감소시키는 관측값은 유지할 수 있다.

가우시안 프로세스의 불확실성은 능동 탐색(Active Exploration)에 직접 활용할 수 있다. 단순히 기하학적으로 탐색되지 않은 공간으로 이동하는 대신 로봇은 예측 분산이 높은 영역을 식별하고 지도 불확실성을 감소시킬 것으로 예상되는 행동을 선택할 수 있다. 정보 이득(Information Gain), 사후 엔트로피 감소(Posterior Entropy Reduction), 분산 감소(Variance Reduction)를 계획 목적함수로 사용할 수 있다. 이를 통해 확률적 매핑과 능동 인지(Active Perception), 자율 데이터 획득(Autonomous Data Acquisition)을 연결할 수 있다.

센서 불확실성과 자세 불확실성(Pose Uncertainty)도 함께 고려해야 한다. 표준 GP 회귀는 일반적으로 학습 데이터의 입력 위치가 정확하게 알려져 있다고 가정하지만, 지도에 측정값을 배치하는 데 사용되는 로봇 자세 자체에도 불확실성이 존재한다. 위치추정 오차는 공간적 관계를 흐리게 만들고 잘못된 신뢰도를 생성할 수 있다. 보다 발전된 방법에서는 불확실한 입력을 전파하거나, 궤적과 지도를 공동 최적화하거나, 자세 공분산(Pose Covariance)에 따라 관측 불확실성을 증가시킬 수 있다.

동적 환경(Dynamic Environment)은 또 다른 문제를 발생시킨다. 표준 공간 가우시안 프로세스는 매핑되는 함수가 비교적 안정적이라고 가정하지만 장애물, 식생, 교통 상황, 기상, 노면 상태는 시간에 따라 변할 수 있다. 입력에 시간(Time)을 추가하면 공분산이 공간적 거리와 시간적 거리 모두에 의존하는 시공간 가우시안 프로세스(Spatiotemporal Gaussian Process)를 구성할 수 있다. 이러한 모델은 지속적으로 유지되는 환경 구조와 운용 중 변화하는 특성을 구분할 수 있다.

야외 자율이동로봇(Outdoor AMR)에서 가우시안 프로세스 지도는 기존의 점유 지도 및 기하학적 지도(Geometric Map)를 완전히 대체하기보다 보완하는 형태로 사용할 수 있다. 점유 격자는 명확한 장애물 구조를 효율적으로 표현하고, GP 계층(GP Layer)은 지형 높이, 거칠기, 슬립 위험(Slip Risk), 무선 통신 연결성(Radio Connectivity), 위치추정 품질(Localization Quality)과 같은 연속적인 물리량을 추정할 수 있다. 이러한 계층을 결합하면 경로 선택, 속도 계획, 위험 인지형 자율 운용에 더욱 풍부한 환경 정보를 제공할 수 있다.

실제 구현에서는 정규화(Normalization), 수치적 안정성(Numerical Stability), 공분산 행렬의 조건 상태(Matrix Conditioning)를 고려해야 한다. 관측 위치가 지나치게 가깝거나 하이퍼파라미터가 적절하지 않으면 커널 행렬이 거의 특이 행렬(Singular Matrix)에 가까워질 수 있다. 작은 대각 지터(Diagonal Jitter)를 추가하고 촐레스키 분해(Cholesky Factorization)와 같은 안정적인 행렬 분해를 사용하며 입력 좌표의 스케일을 조정하고 조건수(Condition Number)를 모니터링하면 학습 및 예측 과정에서 발생하는 수치적 실패를 방지할 수 있다.

검증(Validation)에서는 예측 정확도뿐 아니라 불확실성 보정(Uncertainty Calibration)도 평가해야 한다. 평균 예측이 정확하게 보인다는 이유만으로 해당 지도가 확률적으로 신뢰할 수 있는 것은 아니다. 낮은 불확실성이 부여된 영역은 실제로 높은 불확실성이 부여된 영역보다 작은 오차를 보여야 한다. 교차 검증(Cross-Validation), 제외된 공간 영역(Held-Out Spatial Region), 음의 로그 예측 밀도(Negative Log Predictive Density), 보정 분석(Calibration Analysis), 실제 필드 실험(Field Experiment)을 이용하여 GP의 불확실성이 실제 매핑 오차를 의미 있게 표현하는지 확인할 수 있다.

가우시안 프로세스 매핑은 공간 보간(Spatial Interpolation), 베이지안 추론(Bayesian Inference), 불확실성 인지 로보틱스(Uncertainty-Aware Robotics)를 연결하는 강력한 방법을 제공한다. 측정값은 증거를 제공하고, 커널은 공간적 관계에 대한 가정을 표현하며, 사후 추론은 관측되지 않은 영역을 예측하고, 예측 분산은 아직 알려지지 않은 정도를 정량화한다. 희소, 국부 또는 증분 근사 방법과 결합하면 이러한 프레임워크는 자율 로봇 시스템의 매핑, 탐색, 지형 평가, 위험 인지형 내비게이션(Risk-Aware Navigation)을 지원할 수 있다.

## 02.06. Information Filter and Sparse Extended Information [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

정보 필터(Information Filter)는 가우시안 베이지안 상태 추정(Gaussian Bayesian State Estimation)을 공분산(Covariance)이 아니라 정보(Information)를 이용하여 표현하는 대안적인 방법이다. 칼만 필터(Kalman Filter)가 상태 평균(State Mean) \\(\\mu\\)와 공분산(Covariance) \\(\\Sigma\\)를 유지하는 반면, 정보 필터는 정보 벡터(Information Vector) \\(\\xi\\)와 정보 행렬(Information Matrix) \\(\\Omega\\)을 사용한다. 관련 행렬이 비특이 행렬(Nonsingular Matrix)인 경우 두 표현은 수학적으로 동등하다.

두 표현 사이의 관계는 \\(\\Omega=\\Sigma\^{-1}\\)과 \\(\\xi=\\Omega\\mu\\)로 정의된다. 반대로 상태 추정값은 \\(\\Sigma=\\Omega\^{-1}\\)과 \\(\\mu=\\Sigma\\xi\\)를 통해 복원할 수 있다. 따라서 정보 행렬은 불확실성의 역수(Inverse Uncertainty), 즉 정밀도(Precision)를 나타낸다. 큰 정보값은 상태 변수가 강하게 제약되어 있음을 의미하고, 작은 정보값은 더 큰 불확실성이 존재함을 의미한다.

이러한 표현은 측정 융합(Measurement Fusion)을 직관적으로 해석할 수 있도록 한다. 서로 독립적인 측정값들은 상태에 대한 추가적인 정보를 제공하며, 이러한 정보 기여분은 정보 벡터와 정보 행렬에 직접 더할 수 있는 경우가 많다. 하나의 측정이 정보 증가량(Information Increment) \\(\\Delta\\xi\\)와 \\(\\Delta\\Omega\\)를 제공한다면 갱신은 개념적으로 \\(\\xi\'=\\xi+\\Delta\\xi\\), \\(\\Omega\'=\\Omega+\\Delta\\Omega\\)로 표현할 수 있다.

가우시안 측정 잡음 공분산(Gaussian Measurement Noise Covariance) \\(R\\)을 갖는 선형 측정 모델(Linear Measurement Model) \\(z=Hx+v\\)에서는 측정 정보의 기여분이 \\(H\^TR\^{-1}H\\)와 \\(H\^TR\^{-1}z\\)로 표현된다. 정보 행렬 갱신은 \\(\\Omega\'=\\Omega+H\^TR\^{-1}H\\), 정보 벡터 갱신은 \\(\\xi\'=\\xi+H\^TR\^{-1}z\\)가 된다. 이러한 가산 구조(Additive Structure)는 많은 독립적인 관측값을 결합해야 하는 경우 특히 유용하다.

예측 단계(Prediction Step)는 측정 갱신보다 복잡하다. 운동은 상태 변수 사이에 상관관계(Correlation)를 발생시키며, 동적 모델(Dynamic Model)을 통해 정보를 전파하려면 행렬 역연산(Matrix Inversion)이나 공분산 형태의 예측보다 복잡한 대수 연산이 필요할 수 있다. 따라서 정보 필터가 항상 칼만 필터보다 빠른 것은 아니다. 정보 행렬이 유용한 희소 구조(Sparse Structure)를 가질 때 정보 필터의 장점이 중요해진다.

희소성(Sparsity)은 로봇 매핑(Robot Mapping)과 동시적 위치추정 및 지도작성(SLAM)에서 특히 중요하다. 로봇은 일반적으로 특정 시점에서 전체 랜드마크(Landmark)나 지도 변수(Map Variable) 중 일부만 관측한다. 따라서 측정값은 모든 상태를 서로 연결하기보다 제한된 변수 집합 사이에 직접적인 제약(Direct Constraint)을 생성한다. 정보 형태에서는 이러한 조건부 의존 관계(Conditional Dependency)가 정보 행렬의 0 또는 0에 가까운 블록(Block)으로 나타날 수 있다.

그래프 관점의 해석(Graphical Interpretation)은 확률 그래프 모델(Probabilistic Graphical Model)과 밀접한 관계가 있다. 상태 변수는 노드(Node)로 볼 수 있으며, 0이 아닌 비대각 정보 항(Off-Diagonal Information Term)은 변수 사이의 직접적인 확률적 제약을 나타낸다. 따라서 희소 정보 행렬(Sparse Information Matrix)은 상대적으로 적은 수의 에지(Edge)를 갖는 그래프에 대응한다. 이러한 관계는 재귀적 베이지안 필터링(Recursive Bayesian Filtering), 그래프 모델, 현대적인 SLAM 희소 최적화(Sparse Optimization)를 연결한다.

확장 정보 필터(Extended Information Filter)는 비선형 시스템(Nonlinear System)에 정보 표현을 적용한다. 확장 칼만 필터(Extended Kalman Filter)와 마찬가지로 비선형 운동 및 측정 함수는 국부적인 선형화(Local Linearization)를 통해 근사된다. 자코비안 행렬(Jacobian Matrix)은 상태의 작은 변화가 예측 운동과 관측에 어떤 영향을 주는지를 나타낸다. 이를 통해 얻은 국부 선형 가우시안 근사(Local Linear Gaussian Approximation)를 이용하여 정보 벡터와 정보 행렬을 재귀적으로 갱신할 수 있다.

선형화는 확장 칼만 필터에서 나타나는 것과 동일한 근본적인 한계를 발생시킨다. 현재 추정값이 실제 상태에서 크게 벗어나 있거나 비선형 함수의 곡률(Curvature)이 큰 경우 국부 근사가 부정확해질 수 있다. SLAM에서는 로봇의 방향 오차(Orientation Error)가 여러 랜드마크의 추정 위치에 동시에 영향을 줄 수 있기 때문에 특히 큰 문제가 될 수 있다. 따라서 일관성 있는 초기화(Consistent Initialization)와 적절하게 제어된 선형화가 중요하다.

희소 확장 정보 필터(Sparse Extended Information Filter)는 대규모 SLAM 문제의 구조를 활용하기 위해 개발되었다. 상태에는 로봇 자세(Robot Pose)와 함께 수백 또는 수천 개의 랜드마크가 포함될 수 있으므로 조밀한 공분산 행렬(Dense Covariance Matrix)은 저장 및 갱신 비용이 매우 높다. 반면 직접적이거나 중요한 조건부 의존 관계로 연결된 변수만 명시적인 비영 항(Nonzero Entry)을 필요로 하기 때문에 정보 행렬은 훨씬 더 희소한 형태를 유지할 수 있다.

중요한 문제는 정확한 SLAM 정보 행렬이 모든 연산 과정에서 항상 충분한 희소성을 유지하는 것은 아니라는 점이다. 변수를 제거(Elimination)하거나 과거 로봇 자세를 주변화(Marginalization)하면 이전에는 직접 연결되지 않았던 변수 사이에 새로운 의존 관계가 생성될 수 있다. 이러한 현상을 일반적으로 필인(Fill-In)이라고 하며, 새로운 비영 행렬 요소를 생성하여 점차 계산상의 이점을 감소시킨다. 따라서 유용한 희소성을 유지하려면 근사(Approximation) 또는 신중한 변수 순서 설정(Variable Ordering)이 필요하다.

일반적으로 SEIF라고 불리는 희소 확장 정보 필터(Sparse Extended Information Filter)는 희소 표현을 유지하기 위해 약한 의존 관계(Weak Dependency)를 의도적으로 근사한다. 핵심은 모든 상관관계를 제거하는 것이 아니라 가장 중요한 관계만 명시적으로 유지하는 것이다. 약한 연결을 제거하거나 근사함으로써 활성 연결(Active Connection)의 수를 제한하고 대규모 매핑에서 계산 복잡도를 감소시킬 수 있다.

이러한 근사는 계산 효율성과 통계적 정확성(Statistical Accuracy) 사이의 절충 관계를 발생시킨다. 지나치게 공격적인 희소화(Aggressive Sparsification)는 메모리 사용량과 계산량을 줄일 수 있지만 실제 확률 분포를 왜곡할 수 있다. 반대로 희소화가 충분하지 않으면 보다 정확한 의존 관계를 유지할 수 있지만 정보 행렬이 조밀해진다. 따라서 희소 추정기(Sparse Estimator)는 지도 크기, 계산 자원, 요구 정확도, 허용 가능한 근사 오차 사이의 균형을 고려하여 설계해야 한다.

활성 랜드마크(Active Landmark)는 이러한 구조를 제어하기 위한 하나의 방법을 제공한다. 현재 관측되고 있거나 로봇과 강하게 연결된 랜드마크만 계산 비용이 높은 갱신에 참여시키고, 비활성 랜드마크(Inactive Landmark)는 매번 전체 비용으로 처리하지 않으면서 지도에 계속 유지할 수 있다. 로봇이 이동함에 따라 가시성(Visibility), 거리(Proximity), 정보 가치(Information Value) 또는 다른 기준에 따라 랜드마크가 활성 집합(Active Set)에 포함되거나 제외될 수 있다.

이러한 전략은 로봇이 훨씬 큰 지도 중 국부적인 영역과 상호작용하는 환경에 적합하다. 하나의 관측 주기 동안 카메라 또는 라이다에서 추출된 특징은 일반적으로 주변의 환경 구조만 제약한다. 전체 지도에는 수천 개의 변수가 포함될 수 있지만 측정 갱신은 제한된 국부 영역(Local Neighborhood)에만 영향을 미친다. 희소 표현은 모든 지도 변수를 반복적으로 처리하는 대신 이러한 국부성(Locality)을 활용한다.

정보 형태의 상태 추정(Information-Form Estimation)은 분산형 및 다중 로봇 시스템(Decentralized and Multi-Robot System)에도 개념적으로 적합하다. 서로 다른 로봇이 조건부로 독립적인 관측을 수집하는 경우 각각의 플랫폼은 국부적인 정보 기여분(Local Information Contribution)을 생성하고 이후 이를 결합할 수 있다. 정보 벡터와 정보 행렬은 가산적인 융합(Additive Fusion)을 위한 편리한 표현을 제공할 수 있지만, 로봇 사이의 알려지지 않은 상관관계(Unknown Correlation)는 정보의 중복 계산(Double Counting)을 방지하도록 신중하게 처리해야 한다.

정보 필터링과 팩터 그래프(Factor Graph)의 관계는 현대 SLAM을 이해하는 데 중요하다. 하나의 측정 팩터(Measurement Factor)는 선택된 상태 변수 사이에 확률적 제약을 추가하며, 이는 정보 행렬의 대응 블록에 정보를 추가하는 것과 유사하다. 따라서 팩터 그래프 최적화(Factor-Graph Optimization), 희소 최소제곱법(Sparse Least Squares), 정보 형태 추정은 계산 절차와 갱신 전략에는 차이가 있지만 공통적인 수학적 구조를 공유한다.

현대적인 그래프 기반 SLAM(Graph-Based SLAM)은 고전적인 SEIF 구현보다 스무딩(Smoothing)과 희소 최적화를 사용하는 경우가 많다. 현재의 재귀적 사후 확률만 유지하는 대신 스무딩 방식은 여러 로봇 자세, 랜드마크, 제약 조건을 동시에 최적화한다. 희소 촐레스키 분해(Sparse Cholesky Factorization) 또는 QR 분해(QR Factorization)는 그래프 구조를 효율적으로 활용할 수 있으며, 변수 순서 설정 기법은 필인을 감소시킨다. 그럼에도 정보 필터는 희소성이 중요한 이유를 이해하는 데 여전히 중요한 개념적 기반을 제공한다.

수치적 구현(Numerical Implementation)에서는 행렬 연산을 신중하게 처리해야 한다. 전체 행렬의 역행렬을 명시적으로 계산하는 방식은 일반적으로 피해야 하며, 선형 방정식 풀이(Linear Solve) 또는 행렬 분해(Matrix Factorization)를 통해 필요한 결과를 더 효율적이고 정확하게 얻는 것이 바람직하다. 희소 촐레스키 분해, 희소 QR 기법(Sparse QR Method), 반복 해법(Iterative Solver), 구조화된 블록 연산(Structured Block Operation)은 정보 행렬이 크지만 비영 요소가 상대적으로 적을 때 성능을 크게 향상시킬 수 있다.

양의 정부호성(Positive Definiteness)과 행렬 조건 상태(Matrix Conditioning) 역시 중요하다. 부정확한 센서 모델, 중복된 제약, 잘못된 잡음 가정, 수치적 근사는 시스템을 불안정하게 만들 수 있다. 정규화(Regularization), 신중한 공분산 모델링(Covariance Modeling), 강건한 행렬 분해, 조건수(Condition Number) 모니터링은 신뢰성 있는 상태 추정을 유지하는 데 도움이 된다. 희소화 과정에서도 상태의 관측 가능성(Observability)을 유지하는 핵심 제약이 제거되지 않도록 해야 한다.

관측 가능성은 SLAM에서 직접적인 물리적 의미를 갖는다. 충분한 외부 제약이 없으면 상대적인 위치 관계를 정확하게 추정할 수 있더라도 절대 위치나 방향은 관측 불가능(Unobservable)한 상태로 남을 수 있다. 이러한 게이지 자유도(Gauge Freedom)는 기준 좌표계(Reference Frame)를 고정하거나 적절한 사전 제약(Prior Constraint)을 추가하여 처리해야 한다. 랭크가 부족한 정보 행렬(Rank-Deficient Information Matrix)은 사용 가능한 측정만으로 유일하게 결정할 수 없는 상태 공간의 방향을 나타낼 수 있다.

야외 자율이동로봇(Outdoor AMR)에서는 지도에 많은 랜드마크, 반복적인 주행 궤적, 장시간 관측이 포함될 때 희소 정보 개념이 특히 유용하다. 라이다 특징(LiDAR Feature), 시각 랜드마크(Visual Landmark), 위성항법시스템 제약(GNSS Constraint), 휠 오도메트리(Wheel Odometry), 관성측정장치(IMU) 정보는 규모는 크지만 국부적으로 연결된 상태 추정 문제를 형성한다. 희소성을 활용하면 저장된 모든 변수의 개수에 직접 비례하여 계산하는 대신 실제 환경의 연결 구조에 따라 계산 자원을 사용할 수 있다.

장기 자율운용(Long-Term Autonomy)은 정보 증가를 관리해야 할 필요성을 더욱 높인다. 로봇 자세와 랜드마크를 계속 추가하면 제한 없이 증가하는 추정기는 결국 실용적인 계산 범위를 벗어난다. 주변화, 키프레임 선택(Keyframe Selection), 서브맵(Submapping), 랜드마크 관리(Landmark Management), 희소화, 계층적 표현(Hierarchical Representation)을 이용하여 문제의 크기를 제한할 수 있다. 각각의 방법은 정보를 제거하거나 압축하기 때문에 위치추정과 지도 일관성(Map Consistency)에 가장 중요한 제약을 보존해야 한다.

검증(Validation)에서는 상태 추정 정확성과 구조적 효율성(Structural Efficiency)을 모두 평가해야 한다. 궤적 오차(Trajectory Error)와 랜드마크 오차는 기하학적 성능을 나타내고, 행렬 밀도(Matrix Density), 비영 요소의 개수(Number of Nonzero Entries), 행렬 분해 시간(Factorization Time), 메모리 사용량, 갱신 지연(Update Latency)은 희소성이 실제적인 이점을 제공하는지를 보여준다. 특정 궤적에서는 정확해 보이면서도 체계적으로 과신(Overconfidence)할 수 있기 때문에 일관성 검사(Consistency Test)도 중요하다.

정보 필터는 불확실성을 직접 전파하는 대신 확실성(Certainty)을 축적하는 관점에서 확률적 상태 추정을 바라보는 보완적인 방법을 제공한다. 정보 벡터와 정보 행렬은 대규모 로봇 문제에서 희소하게 나타날 수 있는 조건부 구조를 명확하게 드러낸다. 희소 확장 정보 기법(Sparse Extended Information Method)은 이러한 개념을 비선형 SLAM으로 확장하며, 확률적 의존 관계, 그래프 구조, 근사, 계산 효율성이 확장 가능한 로봇 상태 추정(Scalable Robot State Estimation)에서 서로 밀접하게 연결되어 있음을 보여준다.

## 02.07. Rao Blackwellized Particle Filter for SLAM [w/Code]

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

라오-블랙웰화 파티클 필터(Rao-Blackwellized Particle Filter)는 로봇 궤적 추정(Robot Trajectory Estimation)과 지도 추정(Map Estimation)을 분리하여 동시적 위치추정 및 지도작성(SLAM) 문제를 해결하는 확률적 프레임워크(Probabilistic Framework)이다. 전체 SLAM 상태에 포함된 모든 변수를 샘플링하는 대신 파티클(Particle)을 이용하여 로봇 궤적을 샘플링하고, 각각의 궤적을 조건으로 지도를 계산한다. 이러한 분해는 샘플링 문제의 차원을 감소시키고 SLAM 사후 확률(Posterior)의 구조를 효율적으로 활용한다.

핵심적인 분해(Factorization)는 전체 로봇 궤적을 조건으로 지도를 추정하는 것에서 시작한다. 사후 확률은 개념적으로 \\(p(x_{1:t},m\\mid z_{1:t},u_{1:t})=p(x_{1:t}\\mid z_{1:t},u_{1:t})p(m\\mid x_{1:t},z_{1:t})\\)로 표현할 수 있다. 파티클 필터(Particle Filter)는 궤적 분포(Trajectory Distribution)를 표현하고, 각각의 파티클은 자신의 궤적 가설(Trajectory Hypothesis)을 조건으로 생성된 지도를 함께 유지한다.

이러한 분해 방식을 라오-블랙웰화(Rao-Blackwellization)라고 한다. 일부 불확실한 변수는 명시적으로 샘플링하고, 다른 변수는 샘플링된 변수를 조건으로 적분하거나 해석적으로 추정한다. SLAM에서는 로봇 궤적을 샘플링함으로써 로봇 자세와 지도 변수 사이의 많은 의존 관계를 해소할 수 있다. 이후 특정한 가설 궤적을 기준으로 센서 관측값을 삽입할 수 있기 때문에 지도작성 과정은 상당히 단순해진다.

각각의 파티클은 현재 자세만을 나타내는 것이 아니라 가능한 전체 이력 \\(x_{1:t}\^{[i]}\\)을 표현한다. 이 궤적에는 지도 \\(m\^{[i]}\\)와 파티클 가중치(Particle Weight) \\(w_t\^{[i]}\\)가 연결된다. 따라서 파티클은 로봇의 운동과 누적된 관측을 동시에 설명하는 서로 경쟁하는 가설을 나타낸다. 두 궤적 가설이 서로 달라지면 동일한 센서 측정값도 서로 다른 추정 위치에 삽입되기 때문에 각각의 지도 역시 서로 다르게 발전할 수 있다.

각 시간 단계에서 먼저 제안 분포(Proposal Distribution)를 이용하여 파티클을 전파해야 한다. 기본적인 구현에서는 확률적 모션 모델(Probabilistic Motion Model) \\(p(x_t\\mid x_{t-1},u_t)\\)로부터 새로운 자세를 샘플링한다. 따라서 오도메트리 불확실성(Odometry Uncertainty)은 예상 운동 주변으로 파티클을 확산시킨다. 이 방식은 구현이 간단하지만 새로운 상태를 제안할 때 최신 센서 관측을 이용하지 않으므로 측정 우도(Measurement Likelihood)가 매우 낮은 영역에 파티클을 낭비할 수 있다.

향상된 제안 분포(Improved Proposal Distribution)는 새로운 자세를 생성할 때 최신 관측값을 함께 사용한다. 오도메트리에만 의존하는 대신 스캔 정합(Scan Matching) 또는 다른 국부 정합(Local Registration) 방법을 이용하여 현재 측정값이 해당 파티클의 지도와 잘 정렬되는 자세를 찾을 수 있다. 이후 운동 불확실성과 관측 정보를 결합하여 가능성이 높은 영역에 집중된 제안 분포를 생성한다. 이를 통해 파티클의 불필요한 확산을 줄이고 계산 효율성을 크게 향상시킬 수 있다.

파티클을 전파한 후에는 중요도 가중치(Importance Weight)를 이용하여 각각의 파티클이 최신 관측을 얼마나 잘 설명하는지 평가한다. 개념적으로 예측된 센서 데이터와 실제 측정값이 잘 일치하는 파티클은 더 큰 가중치를 받는다. 정확한 중요도 가중치 계산식은 사용된 제안 분포에 따라 달라진다. 제안 분포가 이미 현재 관측값을 포함하고 있다면 단순히 센서 우도만 적용하는 것이 아니라 해당 제안 분포를 고려하여 가중치를 계산해야 한다.

정규화(Normalization)는 파티클 가중치를 전체 파티클 집합에 대한 확률 분포로 변환한다. 시간이 지나면서 소수의 파티클이 전체 가중치 대부분을 차지하고 나머지 파티클은 거의 의미가 없어질 수 있다. 이러한 현상을 파티클 퇴화(Particle Degeneracy)라고 한다. 무시할 수 있을 정도로 작은 가중치를 갖는 파티클을 계속 전파하면 계산 자원이 낭비되고 불확실성을 표현하는 능력이 감소하기 때문에 재샘플링(Resampling)이 필요하다.

유효 샘플 크기(Effective Sample Size)는 일반적으로 \\(N_{\\text{eff}}=1/\\sum_i(\\tilde{w}_t\^{[i]})\^2\\)로 근사하며, 여기서 \\(\\tilde{w}_t\^{[i]}\\)는 정규화된 가중치이다. 이 값이 전체 파티클 수에 가까우면 가중치가 비교적 균형 있게 분포되어 있음을 의미하고, 작은 값은 가중치가 일부 파티클에 심하게 집중되어 있음을 나타낸다. \\(N_{\\text{eff}}\\)가 특정 임계값 이하로 감소했을 때만 재샘플링을 수행하면 파티클 집합이 건전한 상태에서도 불필요하게 재샘플링하는 것을 방지할 수 있다.

재샘플링 과정에서는 정규화된 가중치에 따라 파티클을 선택한다. 높은 가중치를 갖는 파티클은 여러 번 복제될 수 있고 낮은 가중치를 갖는 파티클은 제거될 수 있다. 단순한 독립 샘플링보다 샘플링 분산(Sampling Variance)이 낮기 때문에 체계적 재샘플링(Systematic Resampling) 또는 저분산 재샘플링(Low-Variance Resampling)이 일반적으로 선호된다. 재샘플링 이후에는 다음 재귀적 갱신을 시작하기 전에 파티클 가중치를 일반적으로 균등하게 초기화한다.

재샘플링은 퇴화 문제를 해결하지만 파티클 빈곤화(Particle Impoverishment)라는 또 다른 문제를 발생시킨다. 성공적인 소수의 파티클을 반복적으로 복제하면 특히 운동 잡음이 작거나 관측이 매우 정보성이 높은 경우 파티클 다양성(Particle Diversity)이 사라질 수 있다. 유용한 대안 가설이 제거되면 이후 이를 다시 복구하기 어렵다. 선택적 재샘플링(Selective Resampling), 향상된 제안 분포, 충분한 운동 불확실성, 복구 메커니즘(Recovery Mechanism)을 이용하여 의미 있는 다양성을 유지할 수 있다.

자세 추정과 가중치 계산 이후에는 각각의 파티클과 연결된 지도를 최신 센서 측정값을 이용하여 갱신한다. 점유 격자 SLAM(Occupancy-Grid SLAM)에서는 레이저 빔이 경로를 따라 자유 공간(Free Space)에 대한 증거를 제공하고 끝점 부근에서는 점유 공간(Occupied Space)에 대한 증거를 제공한다. 각각의 파티클은 서로 다른 궤적 가설을 가지고 있기 때문에 이론적으로 각 파티클이 서로 다른 점유 지도를 유지할 수 있다. 이는 통계적으로 강력하지만 높은 메모리 비용을 요구할 수 있다.

효율적인 구현에서는 파티클이 복제될 때마다 전체 대규모 지도를 복사하는 방식을 피한다. 공유 지도 구조(Shared Map Structure), 쓰기 시 복사(Copy-on-Write), 계층형 격자(Hierarchical Grid), 서브맵(Submap), 트리 기반 표현(Tree-Based Representation)을 이용하면 공통 조상을 가진 파티클들이 변경되지 않은 지도 영역을 공유할 수 있다. 이후 관측에 의해 수정되는 부분만 서로 분리되므로 파티클별 지도를 유지하는 데 필요한 메모리 비용을 크게 감소시킬 수 있다.

스캔 정합은 실제 라오-블랙웰화 SLAM에서 특히 중요하다. 현재 라이다(LiDAR) 스캔을 각 파티클의 기존 지도와 정렬하여 가중치를 계산하기 전에 예측 자세를 정밀하게 보정할 수 있다. 이를 통해 정밀한 센서 기하 정보가 단기 오도메트리 오차를 보정하고 원래의 모션 모델보다 좁은 제안 분포를 생성한다. 따라서 정확한 스캔 정합은 신뢰성 높은 동작에 필요한 파티클의 수를 감소시킬 수 있다.

그러나 스캔 정합을 완벽한 결과로 간주해서는 안 된다. 반복적인 복도 구조, 희박한 야외 환경, 이동 물체, 부족한 기하학적 특징, 부정확한 초기 예측은 잘못된 국부 정렬(Local Alignment)을 생성할 수 있다. 확률적 프레임워크는 하나의 정합 결과로 즉시 수렴하기보다 대안적인 해를 표현할 수 있을 정도의 불확실성을 유지해야 한다. 강건한 스코어링(Robust Scoring)과 운동 사전 정보(Motion Prior)는 이러한 문제를 방지하기 위한 중요한 요소이다.

패스트SLAM(FastSLAM)은 라오-블랙웰화 필터링의 대표적인 사례이다. 랜드마크 기반 FastSLAM에서는 파티클이 로봇 궤적을 나타내고, 각각의 랜드마크 추정은 궤적이 주어졌을 때 조건부 독립(Conditionally Independent)이 된다. 따라서 각 랜드마크는 확장 칼만 필터(Extended Kalman Filter)와 같은 소형 국부 추정기(Local Estimator)를 이용하여 추정할 수 있다. 이를 통해 하나의 거대한 결합 추정 문제를 파티클 기반 궤적 추정과 여러 개의 작은 랜드마크 추정 문제로 변환할 수 있다.

FastSLAM 1.0은 일반적으로 모션 모델을 중심으로 새로운 자세를 샘플링한 후 관측값을 이용하여 파티클의 가중치를 계산한다. FastSLAM 2.0은 로봇 자세를 샘플링할 때 현재 측정값을 포함하도록 제안 분포를 개선한다. 이러한 향상된 제안 분포는 특히 운동 불확실성에 비해 센서가 정확한 경우 샘플링 비효율성을 크게 감소시킨다. 운동과 센싱 모두에서 지지되는 영역에 더 가까운 위치에 파티클이 생성되기 때문이다.

격자 기반 라오-블랙웰화 SLAM(Grid-Based Rao-Blackwellized SLAM)은 동일한 일반 원리를 사용하지만 독립적인 점 랜드마크 대신 점유 지도를 유지한다. GMapping과 같은 알고리즘은 파티클 필터링, 스캔 정합, 적응형 제안 분포(Adaptive Proposal), 선택적 재샘플링, 점유 격자 갱신을 결합하여 널리 활용되었다. 이러한 기술은 비교적 제한된 계산 자원을 갖는 이동 로봇에서도 정확한 2차원 레이저 기반 SLAM을 실용적으로 구현할 수 있도록 했다.

루프 폐쇄(Loop Closure)는 로봇이 이전에 매핑한 영역으로 돌아왔을 때 현재 관측이 과거 측정과 일관된 궤적 가설을 강하게 지지하면서 자연스럽게 발생한다. 누적된 궤적으로 인해 일관되지 않은 지도를 생성한 파티클은 낮은 우도를 받고, 전역적으로 더 높은 일관성(Global Consistency)을 유지한 파티클이 우세해질 수 있다. 그러나 올바른 궤적 가설이 훨씬 이전에 제거되었다면 추가적인 복구 메커니즘 없이는 이후 관측만으로 해당 가설을 자동으로 복원할 수 없다.

이는 파티클 기반 궤적 추정의 근본적인 한계를 보여준다. 가능한 궤적의 수는 시간이 지남에 따라 빠르게 증가하지만 실제 시스템에서는 유한한 수의 파티클만 유지할 수 있다. 따라서 장시간 임무, 심한 환경 모호성(Ambiguity), 부족한 센싱, 큰 누적 드리프트에서는 더 많은 파티클 또는 더욱 강력한 제안 메커니즘이 필요할 수 있다. 파티클 필터는 다중 모드 불확실성(Multimodal Uncertainty)을 표현하는 데 강력하지만 여러 경쟁 궤적이 동시에 높은 가능성을 유지하면 계산 요구량이 증가한다.

데이터 연관(Data Association)은 랜드마크 기반 구현에서 또 다른 중요한 문제이다. 추정기는 새롭게 관측된 특징이 이전에 매핑된 어떤 랜드마크와 대응하는지를 결정해야 한다. 잘못된 연관은 파티클 가중치와 랜드마크 추정 모두를 손상시킬 수 있다. 여러 파티클을 통해 서로 다른 연관 이력을 어느 정도 유지할 수 있지만, 매우 모호한 환경에서는 강건한 게이팅(Robust Gating), 디스크립터 매칭(Descriptor Matching), 일관성 검사(Consistency Check), 연관 불확실성의 명시적인 처리가 필요하다.

야외 자율이동로봇(Outdoor AMR)에서는 라오-블랙웰화 기법을 이용하여 휠 오도메트리(Wheel Odometry), 관성측정장치(IMU), 라이다, 위성항법시스템(GNSS), 비전(Vision), 레이더(Radar) 정보를 결합할 수 있다. 정확한 전역 관측은 궤적 불확실성을 제한하고 국부 스캔 정합은 정밀한 상대 운동 정보를 제공한다. 느슨한 지형에서는 휠 슬립(Wheel Slip)이 운동 제안 분포를 넓힐 수 있으며, 강한 라이다 또는 GNSS 증거는 다시 이를 집중시킬 수 있다. 따라서 센서 신뢰도(Sensor Confidence)는 환경과 운용 조건에 따라 적응적으로 조정해야 한다.

매개변수 조정(Parameter Tuning)은 성능에 큰 영향을 미친다. 운동 잡음은 파티클의 확산 정도를 결정하고, 스캔 정합 매개변수는 제안 분포의 품질에 영향을 주며, 센서 모델 매개변수는 가중치의 민감도를 결정한다. 또한 재샘플링 임계값은 파티클 다양성에 영향을 준다. 불확실성을 지나치게 작게 설정하면 유효한 가설이 제거될 수 있고, 지나치게 크게 설정하면 더 많은 파티클이 필요하다. 따라서 대표적인 궤적, 속도, 노면, 센서 조건, 지도 기하 구조를 이용하여 매개변수를 보정해야 한다.

검증(Validation)에서는 궤적 정확도(Trajectory Accuracy), 지도 일관성(Map Consistency), 파티클 다양성, 유효 샘플 크기, 재샘플링 빈도, 메모리 사용량, 실행 시간을 함께 평가해야 한다. 긴 루프(Long Loop), 반복 구조, 임시 장애물, 센서 성능 저하, 오도메트리 드리프트, 의도적인 위치추정 교란을 포함하여 시험해야 한다. 외관상 깨끗한 지도도 실제로는 일관되지 않은 궤적이나 위험할 정도로 과신된 추정기를 숨기고 있을 수 있으므로 시각적인 지도 품질만으로 성능을 평가해서는 안 된다.

라오-블랙웰화 파티클 필터는 확률적 구조를 활용하여 매우 큰 SLAM 상태 추정 문제를 효율적으로 변환할 수 있음을 보여준다. 복잡한 궤적 변수에 샘플링을 집중하고 특정 궤적이 결정된 이후에는 조건부 지도 추정을 보다 효율적으로 수행한다. 향상된 제안 분포, 스캔 정합, 선택적 재샘플링, 효율적인 지도 표현을 결합함으로써 RBPF 기법은 베이지안 필터링(Bayesian Filtering), 몬테카를로 추정(Monte Carlo Estimation), 실용적인 로봇 지도작성(Practical Robot Mapping)을 연결하는 중요한 방법론을 제공한다.

## 02.08. Uncertainty Propagation in Multi Step Planning [w/Code]

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

자율 로봇(Autonomous Robot)은 하나의 독립된 행동만 수행하는 경우가 거의 없다. 내비게이션(Navigation), 조작(Manipulation), 탐색(Exploration), 검사(Inspection)는 미래까지 영향을 미치는 연속적인 의사결정을 요구한다. 현재의 로봇 상태, 운동 응답, 환경, 센서 관측에는 불확실성(Uncertainty)이 존재하기 때문에 예측되는 모든 미래 상태에도 불확실성이 존재한다. 따라서 다단계 계획(Multi-Step Planning)에서는 결정론적 상태(Deterministic State)만을 이용하여 계획하는 대신 명목 궤적(Nominal Trajectory)과 함께 불확실성을 전파해야 한다.

결정론적 계획기(Deterministic Planner)는 제어 입력 \\(u_0,u_1,\\ldots,u_{T-1}\\)으로부터 상태 시퀀스 \\(x_0,x_1,\\ldots,x_T\\)를 예측한다. 반면 확률적 계획기(Probabilistic Planner)는 가능한 상태에 대한 확률 분포를 나타내는 신념 상태(Belief State) \\(b_t(x)\\)를 기반으로 추론한다. 따라서 계획 문제는 행동, 프로세스 잡음(Process Noise), 관측, 미래의 상태 추정 갱신에 따라 이러한 확률 분포가 어떻게 변화하는지를 예측하는 문제로 확장된다.

확률적 상태 전이 모델(Stochastic Transition Model) \\(p(x_{t+1}\\mid x_t,u_t)\\)에서 불확실성 전파는 베이지안 예측 원리(Bayesian Prediction Principle)를 따른다. 새로운 관측값을 받기 이전의 미래 신념은 \\(b\^-_{t+1}(x_{t+1})=\\int p(x_{t+1}\\mid x_t,u_t)b_t(x_t)dx_t\\)로 표현된다. 현재 가능한 각각의 상태는 여러 미래 상태에 확률을 제공하며, 이에 따라 시스템 동역학(System Dynamics)과 운동 잡음(Motion Noise)에 의해 불확실성이 변화한다.

상태를 가우시안 분포(Gaussian Distribution)로 근사하여 표현하는 경우 불확실성은 평균(Mean) \\(\\mu_t\\)와 공분산(Covariance) \\(\\Sigma_t\\)으로 요약할 수 있다. 프로세스 잡음 공분산(Process-Noise Covariance) \\(Q_t\\)를 갖는 선형 동역학(Linear Dynamics) \\(x_{t+1}=A_tx_t+B_tu_t+w_t\\)에서 공분산은 \\(\\Sigma_{t+1}=A_t\\Sigma_tA_t\^T+Q_t\\)로 전파된다. 이 식은 기존의 불확실성이 동역학에 의해 변환되는 동시에 프로세스 잡음으로부터 새로운 불확실성이 추가되는 과정을 보여준다.

비선형 로봇 시스템(Nonlinear Robotic System)에서는 국부 근사(Local Approximation) 또는 샘플링 방법(Sampling Method)이 필요하다. \\(x_{t+1}=f(x_t,u_t)+w_t\\)인 경우 확장 칼만 필터(Extended Kalman Filter) 방식의 근사는 자코비안(Jacobian) \\(F_t=\\partial f/\\partial x\\)를 이용하여 명목 궤적 주변에서 동역학을 선형화한다. 이후 공분산을 \\(\\Sigma_{t+1}\\approx F_t\\Sigma_tF_t\^T+Q_t\\)로 근사하여 계획된 궤적을 따라 불확실성을 효율적으로 전파할 수 있다.

반복적인 전파는 불확실성을 빠르게 증가시킬 수 있다. 작은 방향 불확실성(Heading Uncertainty)은 처음에는 큰 문제가 없어 보일 수 있지만 수 미터를 전진한 이후에는 상당한 횡방향 위치 불확실성(Lateral Position Uncertainty)을 발생시킬 수 있다. 마찬가지로 속도 불확실성이나 휠 슬립(Wheel Slip)은 여러 계획 단계에 걸쳐 누적될 수 있다. 따라서 불확실성의 기하학적 형태는 운동에 따라 변화하며 미래 분포는 길게 늘어나거나 회전하거나 상태 차원 사이에서 강한 상관관계를 가질 수 있다.

상태 변수들은 독립적으로 변화하지 않기 때문에 상관관계(Correlation)는 매우 중요하다. 위치 불확실성은 방향 불확실성에 의존할 수 있고, 속도 오차는 미래 위치에 영향을 미치며, 지형에 의해 발생하는 슬립은 병진 운동과 방향을 동시에 변화시킬 수 있다. 공분산 행렬(Covariance Matrix)은 비대각 항(Off-Diagonal Term)을 통해 이러한 교차 상관관계(Cross-Correlation)를 표현한다. 이를 무시하면 계획된 궤적에서 로봇이 미래에 존재할 수 있는 영역을 크게 과소평가할 수 있다.

미래의 관측은 불확실성을 감소시킬 수 있으므로 계획에서는 개방 루프 예측(Open-Loop Prediction)과 폐루프 신념 변화(Closed-Loop Belief Evolution)를 구분해야 한다. 개방 루프 전파는 상태 추정을 보정할 수 있는 유용한 측정값이 미래에 존재하지 않는다고 가정하기 때문에 일반적으로 불확실성이 증가한다. 폐루프 계획은 라이다(LiDAR), 비전(Vision), 위성항법시스템(GNSS), 랜드마크(Landmark) 등의 관측이 미래에 제공될 것을 예측하고 예상되는 정보를 미래 신념 상태에 반영한다.

이러한 차이로부터 신념 공간 계획(Belief-Space Planning)이라는 개념이 형성된다. 로봇은 물리적 구성 공간(Configuration Space)만을 따라 계획하는 것이 아니라 추정 상태와 불확실성을 함께 포함하는 공간에서 계획한다. 따라서 동일한 기하학적 목적지에 도달하는 두 개의 궤적도 매우 다른 품질을 가질 수 있다. 하나는 특징이 풍부한 영역을 지나 정확한 위치추정을 유지할 수 있지만, 다른 하나는 모호한 영역을 지나면서 큰 불확실성을 누적할 수 있다.

관측 모델(Observation Model)은 불확실성을 얼마나 감소시킬 수 있는지를 결정한다. 구별 가능한 랜드마크가 포함된 계획된 관측 위치(Viewpoint)는 매우 정보성이 높은 측정값을 제공할 수 있지만, 특징이 거의 없는 긴 복도는 위치추정에 필요한 정보를 거의 제공하지 못할 수 있다. 따라서 불확실성 인지형 계획기(Uncertainty-Aware Planner)는 단순히 기하학적 거리나 이동 시간을 최소화하는 것뿐 아니라 미래의 관측 가능성(Observability)을 향상시키는 행동을 의도적으로 선택할 수 있다.

계획 단계에서는 미래에 실제로 어떤 측정값이 들어올지 알 수 없다. 따라서 가능한 수많은 관측을 모두 고려하지 않는 한 정확한 미래 사후 신념(Future Posterior Belief)을 계산하기 어렵다. 실제 알고리즘에서는 예상 관측(Expected Observation), 최대 우도 관측(Maximum-Likelihood Observation), 예측 피셔 정보(Predicted Fisher Information), 공분산 변화(Covariance Evolution), 샘플링된 측정 시나리오(Sampled Measurement Scenario) 또는 예상 센싱 품질(Expected Sensing Quality)의 다른 표현을 이용하여 이를 근사한다.

가우시안 가정이 적절하지 않은 경우 몬테카를로 전파(Monte Carlo Propagation)를 사용할 수 있다. 현재 신념으로부터 샘플을 생성하고 확률적 동역학을 통해 전파하여 가능한 미래 궤적들의 집합을 생성한다. 이렇게 얻어진 샘플 분포는 비선형 변환(Nonlinear Transformation), 비대칭성(Asymmetry), 다중 모드(Multiple Mode)를 표현할 수 있다. 그러나 샘플 수, 계획 구간, 후보 행동, 환경 복잡도가 증가할수록 계산 비용도 증가한다.

무향 변환(Unscented Transformation)과 시그마 포인트 방법(Sigma-Point Method)은 중간적인 접근 방법을 제공한다. 현재 분포의 평균과 공분산을 표현하는 소수의 결정론적 대표점(Deterministic Representative Point)을 선택하고 이 점들을 비선형 동역학을 통해 전파한다. 이후 결과 상태들을 다시 결합하여 미래 평균과 공분산을 추정한다. 이 방법은 대규모 몬테카를로 샘플을 사용하지 않으면서도 1차 선형화보다 향상된 비선형 근사를 제공할 수 있다.

계획 지평(Planning Horizon)은 불확실성에 큰 영향을 미친다. 짧은 지평의 계획은 여러 번의 불확실한 상태 전이 이후에 발생하는 위험을 과소평가할 수 있고, 지나치게 긴 지평은 예측 오차와 계산 부담을 증가시킨다. 이동 지평 방식(Receding-Horizon Approach)은 여러 단계를 미리 계획한 후 초기 구간만 실행하고 새로운 관측을 반영하여 신념을 갱신한 다음 향상된 상태 추정으로 다시 계획함으로써 이러한 문제를 완화한다.

모델 예측 제어(Model Predictive Control)는 이러한 원리를 자연스럽게 지원한다. 각각의 제어 주기에서 최적화 문제는 유한한 지평에 대해 미래 상태와 불확실성을 예측한다. 제어기는 행동 시퀀스를 선택하고 첫 번째 행동 또는 짧은 구간만 실행한 다음 새로운 센서 정보를 수신하고 다시 최적화 문제를 해결한다. 이러한 반복적인 피드백은 장기적인 예측 오차와 변화하는 환경 조건이 미치는 영향을 제한한다.

충돌 검사(Collision Checking)는 명목 궤적만이 아니라 가능한 로봇 상태의 확률 분포를 고려해야 한다. 명목 경로 자체는 장애물을 통과하지 않더라도 불확실성 분포의 상당 부분이 장애물 경계와 교차할 수 있다. 따라서 공분산에 따라 안전 여유(Safety Margin)를 확대하거나 확률적 충돌 제약(Probabilistic Collision Constraint)을 이용하여 로봇이 위험 영역에 진입할 확률을 명시적으로 제한할 수 있다.

확률 제약 계획(Chance-Constrained Planning)은 이러한 요구사항을 수학적으로 표현한다. \\(P(x_t\\in\\mathcal{X}_{safe})\\geq1-\\epsilon\\)과 같은 제약은 지정된 확률 이상으로 로봇이 안전 영역에 머물도록 요구한다. 위험 허용도(Risk Tolerance) \\(\\epsilon\\)은 계획기가 얼마나 보수적으로 행동할지를 결정한다. 더 작은 값은 높은 신뢰도를 요구하지만 더 긴 경로를 생성하거나 높은 불확실성 조건에서 좁은 통로를 통과하지 못하게 할 수 있다.

불확실성은 동적 실행 가능성(Dynamic Feasibility)에도 영향을 미친다. 지형 마찰(Terrain Friction), 페이로드(Payload), 액추에이터 응답(Actuator Response), 휠-지면 상호작용(Wheel-Ground Interaction)이 불확실하다면 계획기는 모든 명령 가속이나 회전이 정확하게 실행된다고 가정할 수 없다. 강건 궤적 최적화(Robust Trajectory Optimization) 또는 확률적 궤적 최적화(Stochastic Trajectory Optimization)는 여러 실현값을 평가하거나 제약을 강화하고 기대 성능과 분산 및 최악 조건의 거동을 함께 최적화하여 이러한 변동을 고려할 수 있다.

야외 자율이동로봇(Outdoor AMR)은 특히 강한 상태 의존적 불확실성(State-Dependent Uncertainty)에 직면한다. 아스팔트에서는 예측 가능한 운동이 가능하지만 자갈, 진흙, 잔디, 경사면, 연석, 젖은 노면에서는 슬립과 모델 오차가 증가할 수 있다. 지형 인지형 계획기(Terrain-Aware Planner)는 영역마다 서로 다른 프로세스 잡음 모델을 적용할 수 있다. 따라서 안정된 포장도로를 이용하는 약간 긴 경로가 불확실한 지형을 통과하는 짧은 경로보다 낮은 위험의 미래 신념을 생성할 수 있다.

센서 가용성(Sensor Availability) 역시 위치에 따라 달라진다. 건물, 식생, 터널, 지붕이 있는 구조물 주변에서는 GNSS 품질이 저하될 수 있으며, 기하학적 특징이 부족한 개방된 공간에서는 라이다 위치추정 성능이 약해질 수 있다. 비전은 어둠, 눈부심(Glare), 비, 반복적인 장면에서 성능이 저하될 수 있다. 다단계 계획은 이러한 변화를 사전에 고려하여 운동 불확실성이 증가하는 동시에 관측 품질까지 저하되는 궤적을 피할 수 있다.

비용 함수(Cost Function)는 기존의 계획 목적과 불확실성 페널티(Uncertainty Penalty)를 결합할 수 있다. 궤적 비용에는 이동 거리, 이동 시간, 에너지 소비, 부드러움(Smoothness), 충돌 위험, 종단 공분산(Terminal Covariance), 예상 정보 이득(Expected Information Gain)을 포함할 수 있다. 이러한 항목 사이의 가중치는 로봇이 효율성, 위치추정 품질, 안전성, 탐색 중 무엇을 우선하는지를 결정한다. 가중치는 임의의 수치적 편의성이 아니라 임무 요구사항을 반영해야 한다.

불확실성 감소 자체가 가치를 갖게 되면 정보 탐색 행동(Information-Seeking Behavior)이 나타난다. 탐색 과정에서 로봇은 목적지에 직접 가까워지지는 않더라도 랜드마크 또는 알려지지 않은 지형을 더 잘 관측할 수 있는 위치로 의도적으로 이동할 수 있다. 이러한 일시적인 우회는 불확실성을 충분히 감소시켜 이후에 더 안전하고 효율적인 의사결정을 가능하게 한다. 이 원리는 계획을 능동 인지(Active Perception) 및 능동 SLAM(Active SLAM)과 연결한다.

다중 모드 불확실성(Multi-Modal Uncertainty)은 더욱 어려운 문제를 발생시킨다. 서로 다른 여러 로봇 자세, 장애물 운동 또는 환경 가설이 동시에 가능성이 높은 경우 하나의 가우시안 공분산만으로 신념을 정확하게 표현할 수 없다. 파티클 기반 신념 계획(Particle-Based Belief Planning), 혼합 모델(Mixture Model), 시나리오 트리(Scenario Tree), 가설 의존적 계획(Hypothesis-Dependent Planning)이 필요할 수 있다. 계획기는 특정 경로를 선택하기 전에 서로 경쟁하는 가설을 구분하기 위한 행동을 선택해야 하는 경우도 있다.

계산 복잡도(Computational Complexity)는 실제 적용에서 중요한 한계이다. 각각의 후보 행동 시퀀스는 반복적인 상태 예측, 공분산 또는 파티클 전파, 센서 정보 추정, 충돌 평가, 최적화를 요구할 수 있다. 따라서 실시간 시스템에서는 축소 차원의 신념(Reduced-Order Belief), 제한된 계획 지평, 희소 불확실성 표현(Sparse Uncertainty Representation), 병렬 계산(Parallel Computation), 궤적 라이브러리(Trajectory Library), 계층적 계획(Hierarchical Planning) 등의 근사를 이용하여 중요한 의사결정에 계산 자원을 집중한다.

검증(Validation)에서는 예측된 불확실성이 실제 실행 오차와 일치하는지를 평가해야 한다. 동일한 계획 궤적을 반복 실행하여 실제 결과를 예측된 공분산 범위(Covariance Envelope) 또는 충돌 확률과 비교할 수 있다. 센서 누락(Sensor Dropout), 휠 슬립, 지형 변화, 위치추정 성능 저하, 동적 장애물, 모델 불일치(Model Mismatch)를 포함한 시험이 필요하다. 대표적인 운용 조건에서 불확실성 추정이 적절하게 보정되어야 계획기를 신뢰할 수 있다.

불확실성 전파(Uncertainty Propagation)는 계획을 단순한 기하학적 경로 생성(Geometric Path Generation)에서 확률적 의사결정(Probabilistic Decision Making)으로 확장한다. 각각의 행동은 로봇이 어디로 이동할 것으로 예상되는지만 변화시키는 것이 아니라 미래의 상태와 환경에 대해 로봇이 무엇을 얼마나 확실하게 알 수 있는지도 변화시킨다. 운동 불확실성을 전파하고 미래 관측을 예측하며 확률적 안전성을 평가하고 새로운 증거를 바탕으로 반복적으로 재계획함으로써 다단계 계획기는 불완전한 모델, 센서, 환경 지식에서도 신뢰성 높은 자율 행동을 생성할 수 있다.

## 02.09. Probabilistic Map Representations OctoMap [w/Code]

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

확률적 지도 표현(Probabilistic Map Representation)은 공간이 자유 공간(Free Space), 점유 공간(Occupied Space), 미확인 공간(Unknown Space) 중 어떤 상태인지에 대한 불확실성(Uncertainty)을 명시적으로 유지하면서 환경을 표현한다. 각각의 측정값을 절대적인 기하학적 사실로 처리하는 대신 반복적인 관측으로부터 증거를 누적한다. 거리 센서에는 잡음이 존재하고 관측 시점과 시야가 변화하며 물체가 움직이고 환경의 상당 부분이 관측되지 않은 상태로 남을 수 있기 때문에 이러한 방식은 자율 로봇(Autonomous Robot)에서 중요하다.

옥토맵(OctoMap)은 확률적 3차원 점유 매핑(Probabilistic 3D Occupancy Mapping)을 위한 널리 사용되는 프레임워크이다. 옥토맵은 하나의 입방체 영역을 옥턴트(Octant)라고 하는 8개의 더 작은 입방체 영역으로 재귀적으로 분할하는 계층적 데이터 구조(Hierarchical Data Structure)인 옥트리(Octree)를 이용하여 공간을 표현한다. 각각의 리프 노드(Leaf Node)는 일정한 공간 부피를 나타내며 점유 정보를 저장한다. 이러한 구조는 대규모 3차원 환경의 모든 복셀(Voxel)을 동일한 해상도로 유지하는 방식보다 효율적인 표현을 제공한다.

옥트리는 전체 매핑 영역을 포함하는 루트 노드(Root Node)에서 시작한다. 추가적인 공간 세부 정보가 필요한 경우 하나의 노드를 8개의 자식 노드(Child Node)로 분할한다. 이러한 자식 노드는 원하는 지도 해상도(Map Resolution)에 도달할 때까지 다시 재귀적으로 분할할 수 있다. 따라서 상세한 표현이 필요한 영역은 작은 복셀로 표현하고, 넓고 균질하거나 아직 탐색되지 않은 영역은 훨씬 큰 노드로 유지할 수 있다.

이러한 계층적 표현은 환경의 크기와 해상도가 증가할수록 3차원 격자의 크기가 급격하게 증가하기 때문에 특히 유용하다. 조밀한 복셀 격자(Dense Voxel Grid)는 해당 공간에 유용한 정보가 존재하는지와 관계없이 모든 셀에 메모리를 할당한다. 반면 옥트리는 관측을 통해 세부 표현이 필요한 영역을 중심으로 상세 노드를 할당한다. 동일한 점유 상태를 갖는 넓은 영역은 압축할 수도 있으므로 필요한 위치의 정밀한 기하학적 정보를 유지하면서 메모리 사용량을 감소시킬 수 있다.

관측된 각각의 복셀에는 점유 확률(Occupancy Probability) \\(P(n)\\)이 연결되며, 여기서 \\(n\\)은 옥트리 노드를 의미한다. 이 확률은 해당 공간 부피에 장애물이 존재한다고 판단하는 현재의 신념(Belief)을 나타낸다. 1에 가까운 값은 강한 점유 증거를 의미하고, 0에 가까운 값은 강한 자유 공간 증거를 의미한다. 중간값은 불확실성을 나타내며 전혀 관측되지 않은 영역은 명시적으로 미확인 상태로 유지할 수 있다.

옥토맵은 일반적으로 이진 베이즈 필터(Binary Bayes Filter)를 이용하여 점유 상태를 갱신한다. 새로운 센서 관측 \\(z_t\\)가 입력되면 이전의 점유 신념과 역 센서 모델(Inverse Sensor Model)을 결합하여 해당 측정값이 복셀에 점유 공간 또는 자유 공간의 증거를 제공하는지를 판단한다. 따라서 반복되는 측정값은 최신의 단일 관측으로 이전 상태를 덮어쓰는 대신 확률적으로 누적된다.

계산상의 편의를 위해 점유 확률은 로그 오즈(Log-Odds)를 이용하여 표현하는 경우가 많다. 점유 확률이 \\(p\\)일 때 로그 오즈 값은 \\(l=\\log(p/(1-p))\\)로 정의된다. 이를 이용하면 베이지안 갱신(Bayesian Update)을 반복적인 확률 곱셈과 정규화 대신 덧셈 형태로 구현할 수 있다. 점유 관측은 로그 오즈 값을 증가시키고 자유 공간 관측은 로그 오즈 값을 감소시킨다.

역 센서 모델은 거리 측정값이 공간을 어떻게 갱신할지를 결정한다. 라이다(LiDAR) 또는 깊이 센서(Depth Sensor)의 하나의 광선(Ray)을 고려하면 측정된 끝점 이전까지 광선이 통과한 복셀은 자유 공간이라는 증거를 제공하고, 끝점 주변 영역은 점유된 표면이 존재한다는 증거를 제공한다. 따라서 광선 추적(Ray Casting)을 이용하면 하나의 거리 측정값으로 장애물의 점유 상태뿐 아니라 센서와 검출된 물체 사이의 자유 공간도 함께 갱신할 수 있다.

센서 모델은 하나의 관측만으로 절대적인 확신을 부여해서는 안 된다. 라이다 반환값에는 거리 오차, 반사 인공물(Reflective Artifact), 다중 경로 효과(Multipath Effect), 일시적인 물체로부터 발생한 측정값 등이 포함될 수 있다. 마찬가지로 광선이 특정 복셀을 통과했다고 해서 해당 공간이 항상 자유 공간으로 유지된다고 보장할 수 없다. 확률적 갱신을 사용하면 서로 모순되는 관측이 발생하더라도 즉각적인 이진 상태 변화 대신 누적된 신념을 점진적으로 수정할 수 있다.

점유값은 일반적으로 미리 정의된 최소 및 최대 로그 오즈 한계(Log-Odds Limit) 사이로 제한한다. 이러한 제한이 없으면 반복적으로 점유 상태로 관측된 복셀이 지나치게 높은 확신을 축적하여 이후 많은 자유 공간 관측이 들어와도 상태가 쉽게 변경되지 않을 수 있다. 신뢰도의 범위를 제한하면 물체가 이동하거나 초기 관측이 잘못되었거나 장기 운용 과정에서 환경이 변화했을 때 지도가 새로운 관측에 적응할 수 있다.

임계값 처리(Thresholding)는 연속적인 점유 신념을 계획 시스템에서 필요한 분류 상태로 변환한다. 점유 임계값(Occupancy Threshold)보다 높은 확률을 가진 복셀은 장애물로 처리할 수 있으며 충분히 낮은 확률은 자유 공간을 나타낸다. 미확인 공간은 이 두 상태와 별도로 유지된다. 장애물이 관측되지 않았다는 사실이 해당 공간이 자유롭다는 것을 의미하지 않고 단순히 아직 센싱되지 않은 영역일 수도 있기 때문에 이러한 구분은 매우 중요하다.

미확인 공간의 처리(Unknown-Space Handling)는 자율 행동에 직접적인 영향을 준다. 보수적인 로봇은 안전이 중요한 내비게이션을 수행할 때 미확인 복셀을 잠재적인 점유 공간으로 처리할 수 있다. 반면 탐색 로봇(Exploration Robot)은 미확인 영역을 정보 획득(Information Gathering)을 위한 목표로 간주할 수 있다. 따라서 동일한 확률 지도라도 임무 목적, 안전 요구사항, 로봇 동역학(Robot Dynamics), 센싱 능력에 따라 서로 다른 행동을 지원할 수 있다.

옥토맵은 서로 다른 옥트리 깊이(Octree Depth)가 서로 다른 복셀 크기에 대응하기 때문에 다중 해상도 질의(Multi-Resolution Query)를 지원한다. 계획기는 장거리 계획에서는 거친 노드(Coarse Node)를 사용하고 국부 장애물 회피에서는 세밀한 노드(Fine Node)를 사용할 수 있다. 이를 통해 필요한 공간적 세부 수준에 따라 계산 자원을 할당할 수 있다. 다만 여러 해상도를 사용하는 알고리즘에서는 자식 노드로부터 집계된 점유 정보를 신중하게 해석해야 한다.

해상도 선택(Resolution Selection)은 중요한 공학적 절충 관계(Engineering Trade-Off)이다. 작은 복셀은 좁은 장애물, 표면 경계, 작은 통로를 정밀하게 표현할 수 있지만 더 많은 메모리와 계산량을 요구한다. 큰 복셀은 계산 비용을 줄일 수 있지만 장애물을 실제보다 크게 표현하거나 주행 가능한 간격을 제거할 수 있다. 적절한 해상도는 로봇 크기, 정지 거리(Stopping Distance), 센서 정확도, 환경 기하 구조, 지도의 사용 목적을 고려하여 결정해야 한다.

3차원 점유 지도(3D Occupancy Map)는 장애물을 2차원 투영만으로 적절하게 표현하기 어려운 환경에서 특히 유용하다. 돌출 구조물(Overhanging Structure), 경사로(Ramp), 계단, 식생, 하역장(Loading Dock), 선반, 터널, 불규칙한 지형은 동일한 수평 위치에서도 서로 다른 높이를 점유할 수 있다. 3차원 확률 표현을 이용하면 로봇 자체의 물리적 크기와 형태를 기준으로 이러한 구조를 판단할 수 있다.

야외 자율이동로봇(Outdoor AMR)에서는 주행 가능성이 점유 상태뿐 아니라 지형의 기하 구조에 의해서도 결정되기 때문에 이러한 특성이 더욱 중요하다. 점유 상태로 분류된 복셀은 벽, 연석(Curb), 나뭇가지, 차량 또는 지형 표면을 의미할 수 있다. 점유 상태만으로 해당 영역의 주행 가능 여부를 판단할 수는 없다. 따라서 옥토맵은 고도 지도(Elevation Map), 의미론적 레이블(Semantic Label), 표면 법선(Surface Normal), 경사도 추정(Slope Estimation), 지형 분류(Terrain Classification)와 결합하여 더욱 풍부한 내비게이션 판단을 지원할 수 있다.

센서 자세 정확도(Sensor Pose Accuracy)는 지도 품질에 큰 영향을 미친다. 모든 거리 측정값은 추정된 로봇 자세를 이용하여 센서 좌표계(Sensor Frame)에서 지도 좌표계(Map Frame)로 변환해야 한다. 위치추정에 상당한 병진 또는 방향 오차가 존재하면 동일한 표면에 대한 반복적인 관측값이 서로 다른 복셀에 삽입될 수 있다. 이 경우 거리 센서 자체가 정확하더라도 지도가 흐려지거나 두꺼워지고 내부적으로 일관되지 않은 형태가 될 수 있다.

시간에 따른 환경 변화(Temporal Dynamics)는 또 다른 한계를 발생시킨다. 표준 점유 매핑은 주로 누적된 공간 증거를 표현하며 정적 구조물과 이동 물체를 본질적으로 구분하지 않는다. 따라서 차량, 사람, 문, 식생 또는 임시 장비가 오래된 점유 증거를 지도에 남길 수 있다. 변화하는 환경에서는 동적 필터링(Dynamic Filtering), 시간 감쇠(Temporal Decay), 객체 추적(Object Tracking), 지도 계층(Map Layer), 명시적인 시간 의존 점유 모델(Time-Dependent Occupancy Model)이 필요할 수 있다.

지도 갱신에서는 센서 가시성(Sensor Visibility)과 가림(Occlusion)도 고려해야 한다. 장애물의 끝점이 측정되었다고 해서 해당 장애물 뒤쪽 공간에 대한 정보를 얻은 것은 아니다. 광선 기반 갱신은 실제로 관측 가능한 광선 구간만 자유 공간으로 표시한다. 표면 뒤쪽의 관측되지 않은 공간을 자유 공간으로 처리하면 위험한 가정이 지도에 포함되고, 계획기가 실제로 아무런 관측 증거가 없는 영역을 통과하는 경로를 선택할 수 있다.

고속 라이다 및 깊이 센서는 수십만 개에서 수백만 개의 포인트를 생성할 수 있기 때문에 효율적인 삽입(Efficient Insertion)이 중요하다. 각각의 포인트를 독립적으로 직접 처리하면 동일한 광선 갱신이 반복적으로 수행될 수 있다. 포인트 클라우드 필터링(Point-Cloud Filtering), 복셀 다운샘플링(Voxel Downsampling), 최대 거리 처리(Maximum-Range Handling), 광선 키 계산(Ray-Key Computation), 일괄 갱신(Batched Update)을 이용하면 점유 추정에 필요한 정보를 유지하면서 중복 연산을 감소시킬 수 있다.

메모리 효율성은 옥트리 구조 자체뿐 아니라 지도 범위와 관측된 환경의 복잡도에도 영향을 받는다. 세밀한 식생이나 불규칙한 표면을 포함하는 대규모 야외 환경은 여전히 많은 리프 노드를 생성할 수 있다. 가지치기(Pruning)를 통해 동일한 상태를 가진 자식 노드들을 부모 노드로 병합할 수 있으며, 제한된 국부 지도(Bounded Local Map), 서브맵(Submap), 관심 영역(Region of Interest) 전략을 이용하여 장기 운용에서 지도 크기가 무제한으로 증가하는 것을 방지할 수 있다.

옥토맵은 인지(Perception)와 계획(Planning)을 연결하는 인터페이스로 사용할 수 있다. 라이다, 스테레오 카메라(Stereo Camera), RGB-D 센서 또는 재구성된 포인트 클라우드가 공간 측정값을 제공하고, 위치추정(Localization)은 이 측정값들의 전역 위치를 결정하며, 확률적 점유 통합(Probabilistic Occupancy Integration)은 지속적으로 유지되는 환경 표현을 생성한다. 이후 충돌 검사기(Collision Checker)와 경로 계획기(Path Planner)는 로봇 크기의 공간이 점유, 자유, 미확인 상태 중 어디에 해당하는지를 질의할 수 있다.

안전 인지형 내비게이션(Safety-Aware Navigation)에서는 점유 확률과 충돌 확률(Collision Probability)을 동일한 개념으로 간주해서는 안 된다. 점유 확률은 환경 공간에 대한 신념을 나타내지만 충돌 위험은 로봇 자세 불확실성, 로봇 크기, 예측 운동, 장애물 동역학에도 영향을 받는다. 따라서 계획기는 로봇의 기하학적 크기에 따라 점유 영역을 팽창(Inflation)시키거나 점유 지도와 전파된 상태 불확실성 및 확률 제약(Chance Constraint)을 결합할 수 있다.

지도 검증(Map Validation)에서는 시각적인 외형만을 평가해서는 안 된다. 점유 및 자유 공간 분류를 기준 기하 정보(Reference Geometry)와 비교할 수 있으며, 완전성(Completeness)을 이용하여 관련 공간이 어느 정도 관측되었는지 평가할 수 있다. 메모리 사용량, 삽입 속도(Insertion Rate), 질의 지연(Query Latency), 노드 수, 지도 크기는 계산 성능을 나타낸다. 또한 모순된 측정, 이동 물체, 위치추정 드리프트(Localization Drift), 가림, 센서 성능 저하를 포함한 시험이 필요하다.

확률적 3차원 매핑(Probabilistic 3D Mapping)은 로봇이 실제로 관측한 영역, 점유 또는 자유 공간이라고 판단하는 영역, 아직 알려지지 않은 영역을 체계적으로 구분할 수 있는 방법을 제공한다. 옥토맵은 베이지안 점유 갱신(Bayesian Occupancy Update)과 계층적 옥트리 저장 구조(Hierarchical Octree Storage)를 결합하여 조밀하고 균일한 3차원 격자를 사용하지 않고도 상세한 환경 증거를 표현한다. 이러한 특성으로 인해 옥토맵은 복잡한 3차원 환경에서 인지, 매핑, 탐색, 충돌 검사, 자율 내비게이션을 지원하는 중요한 기본 표현 방법이 된다.

## 02.10. Probabilistic Methods in Production Robot Systems

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

:::

실제 운용 로봇 시스템(Production Robot System)은 더 우수한 센서나 결정론적 제어(Deterministic Control)를 사용하더라도 완전히 제거할 수 없는 불확실성(Uncertainty) 속에서 동작한다. 센서 잡음(Sensor Noise), 액추에이터 편차(Actuator Variation), 위치추정 오차(Localization Error), 변화하는 페이로드(Payload), 노면 조건(Surface Condition), 통신 지연(Communication Delay), 이동 물체(Moving Object)는 모두 로봇 운용에 영향을 미친다. 확률적 방법(Probabilistic Method)은 이러한 불확실성을 체계적으로 표현하고 불완전한 관측을 측정 가능한 신뢰도를 가진 의사결정으로 변환하는 방법을 제공한다.

실제 로보틱스(Production Robotics)의 목적은 모든 영역에 확률 이론을 적용하는 것이 아니라 불확실성이 성능이나 안전에 실질적인 영향을 미치는 부분에 확률적 추정(Probabilistic Estimation)을 적용하는 것이다. 따라서 실용적인 아키텍처(Architecture)는 결정론적 실시간 제어(Deterministic Real-Time Control)와 확률적 인지(Probabilistic Perception), 위치추정(Localization), 예측(Prediction), 계획(Planning)을 결합한다. 빠른 내부 제어 루프(Inner Control Loop)는 예측 가능성을 유지하고 상위 계층 모듈은 불확실한 상태와 환경 조건을 추론한다.

상태 추정(State Estimation)은 가장 성숙한 응용 분야 중 하나이다. 휠 오도메트리(Wheel Odometry), 관성 측정(Inertial Measurement), 위성항법시스템(GNSS), 라이다(LiDAR), 카메라(Camera), 엔코더(Encoder) 등의 센서는 로봇 운동에 대해 불완전하고 잡음이 포함된 정보를 제공한다. 칼만 필터(Kalman Filter), 확장 칼만 필터(Extended Kalman Filter), 무향 칼만 필터(Unscented Kalman Filter), 파티클 필터(Particle Filter), 팩터 그래프 추정기(Factor-Graph Estimator)는 이러한 관측을 결합하여 위치, 방향, 속도, 센서 바이어스(Sensor Bias)와 관련 불확실성을 추정한다.

공분산(Covariance)은 신뢰도 정보가 없는 자세 추정만으로는 강건한 자율운용(Robust Autonomy)에 충분하지 않기 때문에 실제 운용에서 중요하다. 수 센티미터 이내로 위치가 추정된 로봇과 위치 불확실성이 수 미터까지 증가한 로봇이 동일하게 행동해서는 안 된다. 실제 시스템에서는 공분산 임계값(Covariance Threshold)을 이용하여 속도를 낮추고, 장애물 안전 여유를 증가시키며, 재위치추정(Relocalization)을 요청하거나, 위치추정 소스를 변경하거나, 제어된 안전 상태(Controlled Safe State)로 전환할 수 있다.

센서 융합(Sensor Fusion)은 모든 측정값이 동일하게 신뢰할 수 있다고 가정하는 대신 센서 유효성(Sensor Validity)을 고려해야 한다. GNSS는 건물 주변이나 터널 내부에서 성능이 저하될 수 있고, 카메라는 눈부심이나 어두운 환경에서 실패할 수 있으며, 휠 오도메트리는 슬립(Slip)이 발생하면 성능이 악화되고, 라이다 정합(LiDAR Matching)은 반복적인 환경에서 모호해질 수 있다. 혁신량 검사(Innovation Test), 잔차 통계(Residual Statistics), 품질 지표(Quality Indicator), 일관성 검사(Consistency Check)를 이용하여 측정값을 수용하거나 가중치를 낮추거나 거부할 수 있다.

확률적 인지(Probabilistic Perception)는 동일한 원리를 객체와 환경 특성으로 확장한다. 검출 시스템은 단순한 범주형 출력만 제공하는 대신 클래스 확률(Class Probability), 위치 공분산(Position Covariance), 추적 신뢰도(Tracking Confidence), 존재 확률(Existence Probability)을 제공할 수 있다. 공간적 불확실성이 큰 검출 객체는 정밀하게 추적되는 장애물과 다른 안전 대응을 요구한다. 따라서 신뢰도는 인지와 후속 계획 모듈 사이의 인터페이스 일부가 된다.

다중 객체 추적(Multi-Object Tracking)은 일반적으로 보행자, 차량, 로봇, 장비의 위치와 속도, 경우에 따라 가속도에 대한 불확실한 추정값을 유지한다. 칼만 계열 필터(Kalman-Family Filter), 상호작용 다중 모델 추정기(Interacting Multiple-Model Estimator), 파티클 기반 방법(Particle Method), 확률적 데이터 연관(Probabilistic Data Association) 기법은 관측 사이에서 객체 운동을 예측할 수 있다. 이렇게 생성된 확률 분포를 이용하면 계획기는 하나의 미래 궤적만 가정하지 않고 객체가 존재할 가능성이 있는 영역을 고려할 수 있다.

데이터 연관(Data Association)은 반복적인 구조나 유사한 객체가 많이 존재하는 실제 운용 환경에서 매우 중요하다. 시스템은 새로운 관측이 기존 추적 객체에 해당하는지, 새로운 객체인지, 또는 클러터(Clutter)인지를 판단해야 한다. 단순 최근접 이웃 연관(Hard Nearest-Neighbor Association)은 계산이 간단하지만 모호한 상황에서는 실패할 수 있다. 게이팅(Gating)과 확률적 연관을 사용하면 잘못된 대응이 추적이나 매핑을 손상시킬 가능성을 줄일 수 있다.

확률적 점유 매핑(Probabilistic Occupancy Mapping)은 실제 운용 시스템에서 또 하나의 중요한 인터페이스를 제공한다. 점유 격자(Occupancy Grid)와 옥토맵(OctoMap) 같은 3차원 표현은 공간이 자유 공간(Free), 점유 공간(Occupied), 미확인 공간(Unknown)인지에 대한 증거를 누적한다. 하나의 측정으로 공간 상태를 영구적으로 결정하는 대신 반복 관측에 따라 점유 신념(Occupancy Belief)을 강화하거나 약화한다. 이를 통해 잡음과 간헐적인 모순 관측을 허용하면서 지속적인 지도작성이 가능하다.

미확인 공간(Unknown Space)은 확인된 자유 공간(Confirmed Free Space)과 반드시 구분해야 한다. 장애물이 관측되지 않았다는 것이 로봇이 해당 영역을 안전하게 통과할 수 있다는 증거는 아니기 때문에 이러한 구분은 특히 안전 측면에서 중요하다. 실제 계획기는 일반 운송 모드에서는 미확인 영역으로의 이동을 금지하면서, 저속 주행과 추가적인 센싱 요구조건이 적용되는 별도의 운용 모드에서는 제한적인 탐색을 허용할 수 있다.

위치추정, 매핑, 인지의 불확실성은 최종적으로 운동 계획(Motion Planning)에 영향을 미친다. 기하학적으로 충돌이 없는 경로도 로봇 자세 불확실성, 장애물 불확실성, 추적 오차를 고려하면 안전하지 않을 수 있다. 불확실성에 따라 안전 여유(Safety Margin)를 조정할 수 있으며, 확률 제약 계획기(Chance-Constrained Planner) 또는 위험 인지형 계획기(Risk-Aware Planner)는 충돌이나 운용 제약을 위반할 확률을 명시적으로 제한할 수 있다.

운동 불확실성(Motion Uncertainty)은 운용 조건에 크게 의존한다. 평탄한 콘크리트 위를 이동하는 자율이동로봇(AMR)은 비교적 예측 가능한 오도메트리를 가질 수 있지만, 자갈, 젖은 포장도로, 경사면, 잔디, 느슨한 노면에서는 슬립과 방향 오차가 증가할 수 있다. 실제 시스템은 지형 클래스(Terrain Class)나 운용 영역별로 서로 다른 프로세스 잡음(Process Noise) 매개변수를 적용하여 운동 예측성이 낮은 영역에서 위치추정과 계획을 더욱 보수적으로 수행할 수 있다.

확률적 모델(Probabilistic Model)은 상태 감시(Health Monitoring)와 예지 정비(Predictive Maintenance)에도 활용할 수 있다. 모터 전류, 온도, 진동, 배터리 거동, 제동 응답, 네트워크 지연, 액추에이터 추종 오차는 정상 운용에서도 변화한다. 통계적 모델(Statistical Model)을 이용하면 정상적인 변동과 비정상적인 거동을 구분하고 성능 저하 가능성을 추정할 수 있다. 이를 통해 단순한 고정 임계값만이 아니라 변화 추세와 신뢰도를 기반으로 유지보수 의사결정을 수행할 수 있다.

그러나 확률적 출력이 결정론적 안전장치(Deterministic Safeguard) 없이 안전 중요 행동(Safety-Critical Action)을 자동으로 실행하도록 해서는 안 된다. 학습된 모델이 특정 위험 상황의 가능성이 낮다고 추정하더라도 기능 안전(Functional Safety)에서는 중요한 한계를 초과했을 때 명확하게 정의된 대응이 필요하다. 따라서 실제 아키텍처에서는 확률적 성능 최적화와 결정론적 안전 강제(Deterministic Safety Enforcement)를 분리하고, 독립적인 안전 한계가 불확실한 자율 의사결정을 무시하고 개입할 수 있도록 설계한다.

이러한 분리는 계층형 아키텍처(Layered Architecture)를 통해 구현할 수 있다. 확률적 인지와 상태 추정은 상태 분포(State Distribution)와 신뢰도 척도를 제공한다. 계획 모듈은 이러한 추정값과 관련 위험을 이용하여 행동을 선택한다. 결정론적 운동 제어(Deterministic Motion Control)는 검증된 명령을 실행하고, 독립적인 안전 계층(Safety Layer)은 속도, 장애물 거리, 비상 입력, 통신 상태 및 기타 중요한 조건을 감시하여 필요할 경우 안전 대응을 강제한다.

실시간 제약(Real-Time Constraint)은 어떤 확률적 방법이 실제 적용 가능한지를 결정한다. 정확 추론(Exact Inference)이 수학적으로 매력적이더라도 계산 시간이 제어 마감시간(Control Deadline)을 초과한다면 사용할 수 없다. 따라서 실제 시스템에서는 제한된 상태 차원, 희소 행렬(Sparse Matrix), 국부 지도(Local Map), 제한된 파티클 수, 고정 계획 지평(Fixed Planning Horizon), 증분 최적화(Incremental Optimization), 하드웨어 가속(Hardware Acceleration), 비동기 처리(Asynchronous Processing)를 이용하여 예측 가능한 지연시간을 유지한다.

시간 자체도 상태 추정 품질의 일부로 취급해야 한다. 매우 정확하지만 지나치게 늦게 도착하는 인지 결과는 약간 덜 정확하더라도 일정한 시간 내에 제공되는 추정값보다 유용하지 않을 수 있다. 따라서 센서 타임스탬프(Sensor Timestamp), 동기화(Synchronization), 처리 지연(Processing Delay), 통신 지연, 현재 시점으로의 예측(Prediction-to-Current-Time)이 중요하다. 상태 추정값은 실제 의사결정이 실행되는 시점과 대응되어야 한다.

보정 품질(Calibration Quality)은 확률적 일관성(Probabilistic Consistency)에 직접적인 영향을 미친다. 잘못된 카메라 내부 파라미터(Camera Intrinsics), 라이다-카메라 외부 파라미터(LiDAR-Camera Extrinsics), 휠 직경, IMU 정렬, GNSS 레버암(GNSS Lever Arm), 타임스탬프 오프셋(Timestamp Offset)은 단순한 무작위 잡음으로 올바르게 표현할 수 없는 체계적 오차(Systematic Error)를 발생시킨다. 실제 시스템에는 통제된 보정 절차, 파라미터 버전 관리(Parameter Versioning), 검증 검사, 운용 중 보정 드리프트(Calibration Drift)를 감지하는 메커니즘이 필요하다.

잡음 매개변수(Noise Parameter) 역시 실제 측정된 거동을 기반으로 설정해야 한다. 임의로 선택된 공분산 값은 추정기를 안정적으로 보이게 만들면서도 잘못된 신뢰도를 생성할 수 있다. 프로세스 잡음과 측정 잡음(Measurement Noise)은 다양한 속도, 페이로드, 노면, 온도, 조명 조건, 센서 구성 등 대표적인 운용 데이터로부터 특성화해야 한다. 이후 잔차 분석(Residual Analysis)을 통해 가정한 불확실성이 실제 관측 오차와 일치하는지 검증할 수 있다.

추정기 일관성(Estimator Consistency)은 평균 정확도만큼 중요하다. 실제로 큰 오차가 발생하면서 작은 공분산을 보고하는 시스템은 과신(Overconfidence) 상태이며, 후속 모듈이 잘못된 확신을 신뢰하기 때문에 위험할 수 있다. 통계적 일관성 검사(Statistical Consistency Test)는 정규화된 추정 오차(Normalized Estimation Error) 또는 혁신량(Innovation)을 예상 분포와 비교한다. 지속적인 불일치는 부정확한 모델, 과소평가된 잡음, 보정 문제 또는 모델링되지 않은 환경 효과를 의미할 수 있다.

실제 운용 로봇에는 명시적인 성능 저하 전략(Degradation Strategy)도 필요하다. 위치추정 신뢰도가 악화되었을 때 시스템이 정상 운용을 무한정 지속해서는 안 된다. 정상 자율운용(Normal Autonomy), 성능 저하 자율운용(Degraded Autonomy), 감속 운행(Reduced Speed), 위치추정 복구(Localization Recovery), 원격 지원(Remote Assistance), 제어 정지(Controlled Stop), 비상 정지(Emergency Stop) 등의 상태를 정의할 수 있다. 전환 기준에는 확률적 신뢰도와 결정론적 상태 지표를 함께 사용하여 시스템 동작을 이해하고 시험할 수 있도록 해야 한다.

고장 모드가 충분히 독립적이라면 중복성(Redundancy)을 통해 강건성을 향상시킬 수 있다. GNSS가 사라진 경우 휠 오도메트리와 IMU는 단기간의 운동 추정을 유지할 수 있으며, 라이다 또는 비전은 환경 기반 보정을 제공할 수 있다. 야외에서는 국부 위치추정이 드리프트된 이후 GNSS를 이용하여 전역 위치를 복구할 수 있다. 확률적 융합은 이러한 정보원을 결합하는 수학적 방법을 제공하지만 시스템 엔지니어링에서는 여전히 공통 원인 고장(Common-Mode Failure)을 식별해야 한다.

다중 로봇 플릿(Multi-Robot Fleet)은 공유 지도, 위치추정 좌표계, 네트워크 가용성, 로봇 사이에서 교환되는 관측에 관한 추가적인 불확실성을 발생시킨다. 다른 로봇에서 전달된 정보는 절대적인 사실로 받아들이기보다 품질 메타데이터(Quality Metadata)를 포함해야 한다. 특히 두 로봇이 동일한 지도나 측정값에 간접적으로 의존하는 경우 상관관계가 발생하며, 분산형 융합(Decentralized Fusion) 과정에서 동일한 정보가 중복 계산(Double Counting)될 위험이 있으므로 주의해야 한다.

로깅(Logging)은 확률적 시스템의 문제를 진단하는 데 필수적이다. 실제 운용 로그에는 최종 자세와 명령뿐 아니라 공분산, 센서 품질 지표, 잔차, 거부된 측정값, 위치추정 모드(Localization Mode), 계획기 위험 지표(Planner Risk Metric), 안전 개입(Safety Intervention), 추정기 리셋(Estimator Reset)도 기록해야 한다. 이러한 기록을 통해 엔지니어는 로봇이 특정 상태를 왜 신뢰했으며 어떤 이유로 특정 행동을 선택했는지를 재구성할 수 있다.

검증(Validation)은 소수의 성공적인 시연이 아니라 반복적인 통계 시험(Statistical Testing)을 요구한다. 다양한 페이로드, 속도, 조명, 기상, 노면 조건, 센서 성능 저하, 네트워크 교란, 동적 장애물 조건에서 동일한 경로를 여러 번 실행해야 한다. 이후 예측된 신뢰 구간(Confidence Interval)을 실제 오차와 비교할 수 있다. 목적은 운용 성능뿐 아니라 시스템이 보고하는 불확실성이 적절하게 보정되어 있는지도 확인하는 것이다.

시나리오 기반 시험(Scenario-Based Testing)은 발생 빈도는 낮지만 결과가 심각할 수 있는 사건을 검증하는 데 특히 유용하다. GNSS 단절, 일시적인 라이다 차단, 카메라 포화(Camera Saturation), 휠 슬립, 이동 군중, 지도 변화, 위치추정 점프(Localization Jump), 패킷 지연, 부분적인 센서 고장을 체계적으로 주입할 수 있다. 로봇은 복구 성능뿐 아니라 불확실성이 증가하는 동안 적절하게 신뢰도를 낮추고 안전한 행동 상태로 전환할 수 있음을 보여야 한다.

시뮬레이션(Simulation)과 디지털 트윈(Digital Twin)은 실제 환경에서 반복하기 어려운 조건까지 시험 범위를 확장할 수 있다. 몬테카를로 실험(Monte Carlo Experiment)을 이용하여 수천 번의 시험에서 센서 잡음, 마찰, 지연시간, 장애물 운동, 부품 매개변수를 변화시킬 수 있다. 시뮬레이션이 실제 필드 검증(Field Validation)을 대체할 수는 없지만 결정론적인 정상 조건 시험에서 발견하기 어려운 불확실성의 조합을 찾아내고 비용이 높은 실제 시험의 우선순위를 결정하는 데 도움을 줄 수 있다.

확률값은 최종적으로 실제 운용 의사결정으로 변환되어야 한다. 이를 위해서는 서로 다른 행동에 대해 어느 정도의 불확실성을 허용할 수 있는지 명확한 정책을 정의해야 한다. 고속 주행, 도킹(Docking), 좁은 통로 주행, 조작 작업, 사람과의 상호작용은 서로 다른 신뢰도 임계값을 요구할 수 있다. 따라서 동일한 추정기 출력이라도 현재 임무의 위험도와 정밀도 요구사항에 따라 서로 다른 행동으로 이어질 수 있다.

배치 이후에도 이러한 임계값을 지속적으로 모니터링하는 것이 중요하다. 환경 조건과 하드웨어 특성은 시간이 지나면서 변화하기 때문에 개발 단계에서 보정된 모델이 이후에는 정확하지 않을 수 있다. 플릿 데이터(Fleet Data)를 통해 잔차 분포의 변화, 위치추정 신뢰도, 개입 빈도, 센서 신뢰성의 변화를 확인할 수 있다. 이러한 증거를 기반으로 검증되지 않은 임의의 매개변수 변경이 아니라 통제된 재보정(Recalibration)과 모델 갱신을 수행할 수 있다.

확률적 방법은 불확실성이 시스템 경계를 넘어 공유되는 명시적인 공학적 신호(Engineering Signal)가 될 때 실제 운용에서 가장 큰 가치를 제공한다. 센서는 측정값과 품질 지표를 제공하고, 추정기는 상태와 공분산을 생성하며, 인지는 신뢰도를 보고하고, 계획기는 위험을 평가하며, 안전 메커니즘(Safety Mechanism)은 강제적인 한계를 적용한다. 이를 통해 불확실한 물리적 관측에서 실제 로봇 행동까지 추적 가능한 동작 구조를 구축할 수 있다.

따라서 실제 운용이 가능한 확률적 아키텍처(Production-Ready Probabilistic Architecture)는 수학적 추정과 결정론적 공학 규율(Deterministic Engineering Discipline)을 결합해야 한다. 목표는 가장 복잡한 확률적 기법을 사용하는 것이 아니라 보정된 불확실성(Calibrated Uncertainty), 제한된 계산량(Bounded Computation), 관측 가능한 고장 모드(Observable Failure Mode), 검증된 임계값(Validated Threshold), 안전한 성능 저하(Safe Degradation)를 확보하는 것이다. 이러한 원칙을 인지, 위치추정, 매핑, 계획, 제어, 플릿 운용 전체에 통합하면 불확실성을 통제되지 않는 고장 원인이 아니라 관리 가능한 시스템 특성으로 전환할 수 있다.
