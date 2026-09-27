**Volume 13. Mapping Localization and SLAM**

# Chapter 03. 2D SLAM

## 03.01. 2D SLAM Problem Formulation and Graph SLAM Theory

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

2차원 동시적 위치추정 및 지도작성(2D Simultaneous Localization and Mapping, 2D SLAM)은 이동 로봇(Mobile Robot)의 궤적(Trajectory)을 추정하는 동시에, 처음에는 알려지지 않은 환경의 지도(Map)를 구축하는 문제를 다룬다. 로봇은 휠 인코더(Wheel Encoder), 관성 센서(Inertial Sensor), 2D 라이다(2D LiDAR) 등의 센서를 통해 부분적이고 잡음이 포함된 정보만을 관측한다. 위치추정(Localization)은 지도에 의존하고 지도작성(Mapping)은 위치추정에 의존하므로, 두 추정 문제는 본질적으로 서로 결합되어 있다.

일반적인 2D SLAM 상태(State)는 로봇 자세(Pose)를 \\(x_t=[x_t,y_t,\\theta_t]\^T\\)로 표현하며, 여기서 \\(x_t\\)와 \\(y_t\\)는 평면상의 위치를 나타내고 \\(\\theta_t\\)는 방향각(Heading)을 나타낸다. 시간에 따른 전체 궤적은 \\(X=\\{x_0,x_1,\\ldots,x_T\\}\\)로 표현할 수 있다. 이 궤적을 따라 수집된 측정값(Measurement)은 서로 다른 로봇 자세와 환경에서 관측 가능한 구조 사이의 기하학적 제약조건(Geometric Constraint)을 제공한다.

SLAM의 확률론적 정식화(Probabilistic Formulation)는 제어 입력(Control Input)과 센서 관측(Sensor Observation)이 주어졌을 때 로봇 상태와 지도 변수(Map Variable)의 사후확률분포(Posterior Distribution)를 추정하는 것을 목표로 한다. 개념적으로 이는 \\(p(X,M\|Z,U)\\)로 표현할 수 있으며, 여기서 \\(X\\)는 궤적, \\(M\\)은 지도, \\(Z\\)는 관측값, \\(U\\)는 운동 정보(Motion Information)를 의미한다. 실제 시스템에서는 궤적이 증가할수록 전체 결합확률(Joint Probability)을 직접 유지하는 계산 비용이 증가하기 때문에 이를 근사하여 처리한다.

그래프 SLAM(Graph SLAM)은 이러한 추정 문제를 구조화된 최적화 문제(Structured Optimization Problem)로 변환한다. 로봇 자세는 그래프(Graph)의 노드(Node)로 표현되고, 오도메트리(Odometry), 스캔 정합(Scan Matching), 랜드마크(Landmark), 루프 폐쇄(Loop Closure) 등에서 얻은 공간적 관계는 엣지(Edge)가 된다. 각 엣지는 두 노드 사이의 상대 자세 측정(Relative-Pose Measurement)과 그 측정의 불확실성(Uncertainty)을 함께 표현한다. 결과적으로 자세 그래프(Pose Graph)는 누적된 공간 제약조건을 간결하게 표현한다.

오도메트리 엣지(Odometry Edge)는 일반적으로 연속된 자세를 연결하고 시간 단계(Time Step) 사이에서 추정된 상대 운동(Relative Motion)을 나타낸다. 휠 인코더, 시각 오도메트리(Visual Odometry), 라이다 오도메트리(LiDAR Odometry), 또는 융합된 관성 측정(Fused Inertial Measurement)이 이러한 제약조건을 제공할 수 있다. 모든 측정에는 오차가 포함되므로 이를 반복적으로 적분하면 드리프트(Drift)가 발생한다. 자세 그래프는 초기 궤적이 정확하다고 가정하지 않고 측정값을 제약조건으로 유지하여 이후 전체적으로 최적화할 수 있도록 한다.

자세 \\(x_i\\)와 \\(x_j\\)에 대해 엣지 측정값 \\(z_{ij}\\)는 두 자세 사이에서 관측된 상대 변환(Relative Transformation)을 나타낸다. 현재 자세 추정값으로부터 예측값 \\(\\hat{z}_{ij}\\)를 계산할 수 있으며, 측정값과 예측값의 차이를 통해 오차 벡터(Error Vector) \\(e_{ij}\\)가 정의된다. 그래프 최적화(Graph Optimization)는 예측된 상대 변환들의 집합이 실제 측정된 상대 변환과 가능한 한 일관성을 갖도록 각 자세를 조정한다.

측정 불확실성(Measurement Uncertainty)은 공분산 행렬(Covariance Matrix) 또는 그 역행렬인 정보 행렬(Information Matrix) \\(\\Omega_{ij}\\)로 표현된다. 신뢰성이 높은 측정값은 불확실성이 큰 측정값보다 최적화 과정에서 더 강한 영향을 갖는다. 전체 목적함수(Global Objective)는 일반적으로 가중 비선형 최소제곱 문제(Weighted Nonlinear Least-Squares Problem)로 표현되며, 그래프의 모든 엣지에 대해 \\(\\sum e_{ij}\^{T}\\Omega_{ij}e_{ij}\\)와 같은 비용함수(Cost Function)를 최소화한다.

평면상의 로봇 자세는 강체 변환군(Rigid-Body Transformation Group) SE(2)에 속하기 때문에 자세 연산을 일반적인 벡터 덧셈과 동일하게 처리할 수 없다. 하나의 자세는 병진(Translation)과 회전(Rotation)을 함께 포함하며, 상대 변환에는 합성(Composition)과 역변환(Inversion) 연산이 필요하다. 따라서 그래프 SLAM 구현에서는 잔차(Residual), 야코비안(Jacobian), 자세 갱신(Pose Update), 로봇 좌표계(Robot Frame)와 전역 지도 좌표계(Global Map Frame) 사이의 좌표 변환을 계산할 때 적절한 SE(2) 기하학(Geometry)을 적용한다.

회전 성분이 병진 좌표의 변환에 영향을 주기 때문에 최적화 문제는 비선형(Nonlinear) 특성을 갖는다. 가우스-뉴턴(Gauss-Newton)과 레벤버그-마쿼트(Levenberg-Marquardt) 등의 알고리즘은 현재 추정값 주변에서 오차 함수를 반복적으로 선형화(Linearization)하고, 생성된 희소 선형 시스템(Sparse Linear System)을 해결한 뒤 자세를 갱신한다. 비선형 최적화가 느리게 수렴하거나 잘못된 국소해(Local Solution)에 수렴하는 것을 방지하기 위해서는 충분히 합리적인 초기 궤적(Initial Trajectory)이 중요하다.

희소성(Sparsity)은 그래프 SLAM이 갖는 주요 계산상의 장점 중 하나이다. 각각의 측정은 일반적으로 소수의 상태만을 연결하기 때문에 생성되는 헤시안 행렬(Hessian Matrix) 또는 정보 행렬에는 많은 영값 블록(Zero-Valued Block)이 존재한다. 희소 행렬 기법(Sparse Matrix Technique)은 이러한 구조를 활용하여 전체 추정 문제를 밀집 행렬(Dense Representation)로 처리하는 방법보다 수천 개 또는 수백만 개의 제약조건을 포함하는 궤적을 효율적으로 최적화할 수 있도록 한다.

루프 폐쇄(Loop Closure)는 시간적으로 멀리 떨어져 있는 자세 사이에 새로운 제약조건을 추가하기 때문에 특히 중요하다. 로봇이 이전에 방문했던 장소를 다시 인식하면 새로운 엣지를 통해 누적된 궤적 드리프트를 확인할 수 있다. 이후 전역 그래프 최적화(Global Graph Optimization)는 하나의 위치에서 불연속적으로 보정하는 대신 여러 자세에 걸쳐 보정량을 분산시켜 기하학적으로 더욱 일관된 궤적과 지도를 생성한다.

그러나 루프 폐쇄는 매우 신중하게 검증되어야 한다. 잘못된 루프 폐쇄(False Loop Closure)는 강력하지만 잘못된 제약조건을 추가하여 전체 자세 그래프를 왜곡할 수 있다. 따라서 실제 시스템은 장소 인식(Place Recognition)에 스캔 정렬(Scan Alignment), 특징 일관성(Feature Consistency), 변환 타당성(Transformation Plausibility) 등의 기하학적 검증(Geometric Verification)을 결합한다. 또한 강건 손실 함수(Robust Loss Function)를 사용하면 대부분의 제약조건과 일치하지 않는 측정값의 영향을 감소시킬 수 있다.

라이다 기반 2D SLAM(LiDAR-Based 2D SLAM)에서는 스캔 정합(Scan Matching)이 상대 자세 제약조건을 제공하는 경우가 많다. 연속하거나 서로 인접한 레이저 스캔(Laser Scan)은 관측된 표면 사이의 기하학적 일치도를 최대화하는 변환을 추정하여 정렬된다. 점대점 거리(Point-to-Point Distance), 점대선 거리(Point-to-Line Distance), 상관관계(Correlation), 우도장(Likelihood Field), 점유 표현(Occupancy Representation) 등에 기반한 방법을 사용할 수 있다. 추정된 변환과 불확실성은 이후 자세 그래프의 엣지가 될 수 있다.

자세 그래프(Pose Graph)와 환경 지도(Environmental Map)는 개념적으로 구분할 필요가 있다. 그래프는 로봇 상태 사이의 관계를 표현하는 반면, 점유 격자(Occupancy Grid)는 공간 셀(Cell)이 점유되었거나 비어 있을 확률을 표현한다. 궤적 최적화 이후 저장된 레이저 관측값을 보정된 자세를 이용하여 다시 변환하고 격자에 재통합할 수 있다. 따라서 궤적의 일관성이 향상되면 벽의 경계가 더욱 선명해지고 환경 구조도 더욱 정확하게 정렬된다.

게이지 자유도(Gauge Freedom)는 상대 측정값만으로는 절대적인 전역 좌표계(Global Coordinate System)를 결정할 수 없기 때문에 발생한다. 모든 자세를 동시에 동일하게 이동하거나 회전하더라도 상대 제약조건은 변하지 않는다. 따라서 실제 최적화기(Optimizer)는 일반적으로 초기 자세와 같은 하나의 자세를 고정하거나 적절한 사전 제약조건(Prior Constraint)을 추가한다. 이를 통해 기준 좌표계(Reference Frame)를 설정하고 최적화 문제가 수학적으로 미결정 상태(Underdetermined)가 되는 것을 방지한다.

2D SLAM 시스템의 품질은 최적화기뿐만 아니라 제약조건의 품질과 관측 가능성(Observability)에 의해서도 결정된다. 긴 복도, 반복적인 구조, 넓은 개방 공간, 이동 객체(Moving Object), 휠 슬립(Wheel Slip), 희소한 기하 구조(Sparse Geometry), 빠른 회전은 위치추정 성능을 저하시킬 수 있다. 따라서 센서 융합(Sensor Fusion), 운동 사전정보(Motion Prior), 스캔 품질 평가(Scan-Quality Evaluation), 강건 추정(Robust Estimation), 신중한 루프 폐쇄 검증이 신뢰성 높은 시스템의 핵심 요소가 된다.

그래프 SLAM은 프런트엔드 인지(Front-End Perception)와 백엔드 추정(Back-End Estimation)을 자연스럽게 분리한다. 프런트엔드는 센서 데이터를 처리하고 스캔 정합을 수행하며 루프 후보(Loop Candidate)를 탐지하고 제약조건을 생성한다. 백엔드는 그래프를 유지하고 전역 최적화 문제를 해결한다. 이러한 분리는 서로 다른 센서와 정합 알고리즘이 공통 추정 아키텍처(Common Estimation Architecture)에 연결되면서도 궤적 보정을 위한 수학적으로 일관된 프레임워크를 유지할 수 있도록 한다.

자율이동로봇(Autonomous Mobile Robot, AMR)에서 이러한 정식화는 지역 운동 추정(Local Motion Estimation)과 전역적으로 일관된 내비게이션(Globally Consistent Navigation)을 연결하는 효과적인 기반을 제공한다. 단기 오도메트리는 연속적인 이동을 지원하고, 지역 스캔 정합(Local Scan Matching)은 점진적인 오차를 제한하며, 루프 폐쇄는 장기 드리프트(Long-Term Drift)를 제어한다. 그래프 최적화는 이러한 정보원을 각각의 불확실성에 따라 결합하여 점유 지도작성, 위치추정, 경로 계획(Path Planning), 반복적인 자율 운용을 지원할 수 있는 궤적을 생성한다.

산업 현장 적용(Industrial Deployment)에서는 이론적 정식화가 제한된 컴퓨팅 자원(Bounded Computing Resource)과 실시간 제약조건(Real-Time Constraint) 아래에서 동작해야 한다. 모든 센서 프레임마다 노드를 생성하는 대신 키프레임(Keyframe)을 선택할 수 있으며, 중복 제약조건(Redundant Constraint)을 줄이고 최적화를 증분 방식(Incremental) 또는 비동기 방식(Asynchronous)으로 수행할 수 있다. 이러한 공학적 선택은 메모리 사용량, 지연시간(Latency), 계산 복잡도(Computational Complexity)를 제어하면서 그래프 SLAM의 핵심 구조를 유지한다.

따라서 2D 그래프 SLAM(2D Graph SLAM)의 핵심 개념은 단순히 로봇의 운동을 시간 순서에 따라 앞으로 적분하는 것이 아니라, 공간적 관계를 이후 수정 가능한 제약조건(Revisable Constraint)으로 유지하는 데 있다. 새로운 관측은 과거 운동에 대한 해석을 변경할 수 있으며, 루프 폐쇄는 긴 궤적에 걸쳐 누적된 오차를 보정할 수 있다. 불확실성을 명시적으로 표현하고 관련된 모든 제약조건을 함께 최적화함으로써 그래프 SLAM은 전역적으로 일관된 평면 지도작성과 위치추정의 이론적 기반을 제공한다.

## 03.02. GMapping Particle Filter SLAM Implementation [w/Code]

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

GMapping은 주로 평면 레이저 거리 센서(Planar Laser Range Sensor)를 장착한 이동 로봇(Mobile Robot)을 위해 설계된 대표적인 2D SLAM 기법이다. 핵심 알고리즘은 라오-블랙웰화 파티클 필터(Rao-Blackwellized Particle Filter, RBPF)를 기반으로 하며, 파티클(Particle)을 이용해 로봇 궤적(Robot Trajectory)을 추정하는 동시에 각 궤적 가설(Trajectory Hypothesis)에 대응하는 별도의 점유 격자 지도(Occupancy-Grid Map)를 유지한다. 이러한 분해 방식은 SLAM 문제를 계산 가능한 형태로 만들며 실제 실내 로봇 지도작성에서 역사적으로 중요한 역할을 해왔다.

GMapping의 이론적 기반은 SLAM 사후확률(Posterior)을 로봇 궤적에 대한 확률분포와 지도에 대한 조건부 확률분포(Conditional Distribution)로 분해하는 것에서 시작한다. 모든 로봇 자세와 지도 셀(Map Cell)을 동시에 추정하는 대신 파티클 필터(Particle Filter)가 가능한 궤적을 샘플링한다. 특정 궤적과 이에 대응하는 레이저 관측값이 주어지면 점유 지도(Occupancy Map)를 조건부로 추정할 수 있으므로 확률적 추정 문제의 차원을 감소시킬 수 있다.

각 파티클은 로봇의 전체 궤적에 대한 하나의 가설을 나타내며, 해당 궤적을 따라 수집된 관측값으로 구축된 지도를 함께 포함한다. 로봇이 이동하면 오도메트리(Odometry) 정보를 사용하는 운동 모델(Motion Model)에 따라 파티클이 전파된다. 이후 레이저 스캔(Laser Scan)을 각 파티클의 지도와 비교하여 평가하고, 각 궤적 가설이 현재 센서 관측을 얼마나 잘 설명하는지를 나타내는 우도값(Likelihood)을 계산한다.

주로 오도메트리 운동 모델에 의존하는 단순한 파티클 필터는 휠 오도메트리(Wheel Odometry)의 불확실성이 빠르게 누적되기 때문에 많은 수의 파티클을 요구할 수 있다. GMapping은 최신 레이저 관측값을 제안 분포(Proposal Distribution)에 포함하여 이 문제를 완화한다. 스캔 정합(Scan Matching)을 사용해 현재 스캔을 기존 지도와 더욱 잘 정렬하는 자세를 찾고, 파티클을 오도메트리 불확실성에 따라 넓게 분산시키는 대신 측정 우도가 높은 영역 주변에 집중시킨다.

이러한 개선된 제안 분포(Improved Proposal Distribution)는 GMapping의 가장 중요한 특징 가운데 하나이다. 알고리즘은 운동 추정(Motion Estimate), 현재 레이저 스캔, 기존 지도를 결합하여 지역적으로 정보를 반영한 자세 제안(Pose Proposal)을 생성한다. 따라서 타당한 궤적을 표현하기 위해 필요한 파티클 수를 줄일 수 있으며, 구조화된 실내 환경과 중간 규모 환경에서 유용한 지도 정확도를 유지하면서 계산 비용을 상당히 감소시킬 수 있다.

스캔 정합은 측정된 레이저 종단점(Laser Endpoint)과 지도에 이미 표현된 점유 구조(Occupied Structure) 사이의 일치도를 최대화하는 로봇 자세를 탐색한다. 일반적으로 오도메트리 추정값을 초기 탐색 위치로 사용하고 주변의 병진(Translation) 및 회전(Rotation) 후보를 평가한다. 성공적인 정합은 개선된 자세 추정값을 생성하며, 파티클 샘플링에 활용할 수 있는 지역 자세 불확실성(Local Pose Uncertainty)에 관한 정보도 제공한다.

새로운 자세가 제안되면 각 파티클에는 해당 가설에서 현재 관측이 발생할 확률을 반영하는 중요도 가중치(Importance Weight)가 할당된다. 개념적으로 예측된 지도와 실제 레이저 스캔이 높은 수준으로 일치하는 파티클에는 큰 가중치가 부여되고, 일관성이 낮은 가설에는 작은 가중치가 부여된다. 이러한 가중치를 정규화(Normalization)하면 현재 파티클 집합으로 표현되는 사후확률분포의 이산 근사(Discrete Approximation)를 구성할 수 있다.

가중치 부여가 반복되면 대부분의 파티클 가중치가 매우 작아지고 소수의 파티클만 사후확률을 실질적으로 표현하는 파티클 퇴화(Particle Degeneracy)가 발생한다. 따라서 파티클 필터 SLAM에서는 높은 가중치의 파티클을 복제하고 낮은 가중치의 파티클을 제거하는 재샘플링(Resampling)이 필요하다. 그러나 지나치게 빈번한 재샘플링은 유용한 궤적 다양성(Trajectory Diversity)을 제거하여, 특히 관측 가능한 기하학적 정보가 부족할 때 파티클 빈곤화(Particle Impoverishment)를 유발할 수 있다.

GMapping은 이러한 문제를 줄이기 위해 적응형 재샘플링(Adaptive Resampling)을 사용한다. 대표적인 판단 기준은 유효 표본 크기(Effective Sample Size)이며, 일반적으로 정규화된 파티클 가중치 제곱합의 역수와 관련된다. 파티클 가중치가 비교적 균형을 유지하면 재샘플링을 연기할 수 있으며, 유효 파티클 수가 특정 임계값(Threshold) 아래로 감소하면 재샘플링을 수행한다. 이를 통해 파티클의 다양성을 더 오래 유지하면서 불필요한 계산과 과도한 파티클 복제를 줄일 수 있다.

각 파티클과 연관된 점유 격자 지도는 환경을 자유 공간(Free), 점유 공간(Occupied), 미확인 공간(Unknown)을 나타내는 이산 셀(Discrete Cell)로 분할한다. 레이저 빔(Laser Beam)은 이러한 상태에 관한 관측 증거를 제공한다. 유효한 빔이 통과한 셀은 일반적으로 자유 공간 방향으로 갱신되고 장애물에 대응하는 빔의 종단점은 점유 공간 방향으로 갱신된다. 반복적인 관측을 통해 벽, 가구, 설비 및 이동 가능한 영역의 확률적 표현이 점진적으로 정교해진다.

지도 해상도(Map Resolution)는 정확도와 계산 요구량 모두에 큰 영향을 준다. 작은 격자 셀은 환경의 기하 구조를 더욱 정밀하게 표현할 수 있지만 더 많은 메모리를 요구하며 스캔 정합과 지도 갱신 비용을 증가시킨다. 큰 셀은 계산량을 줄일 수 있지만 좁은 통로, 벽 경계, 작은 장애물을 흐릿하게 표현할 수 있다. 따라서 실제 GMapping 설정에서는 라이다 특성, 환경 규모, 내비게이션 요구조건, 사용 가능한 컴퓨팅 자원을 고려하여 해상도를 선택한다.

레이저 스캔 정합이 상당한 운동 오차를 보정하더라도 오도메트리는 여전히 중요하다. 운동 모델은 스캔 사이의 병진 및 회전에 대한 사전 추정(Prior Estimate)을 제공하고 파티클 전파 과정에서 불확실성이 어떻게 확산되는지를 결정한다. 병진 잡음(Translational Noise), 회전 잡음(Rotational Noise), 그리고 이들의 결합 관계를 설명하는 파라미터는 실제 로봇 플랫폼의 물리적 특성을 적절하게 반영해야 한다. 부적절하게 조정된 운동 불확실성은 지나친 파티클 분산이나 부정확한 오도메트리에 대한 과도한 신뢰를 발생시킬 수 있다.

레이저 센서 품질(Laser Sensor Quality) 역시 지도작성 성능에 직접적인 영향을 준다. 정확한 각도 보정(Angular Calibration), 신뢰할 수 있는 거리 측정, 적절한 최소 및 최대 측정거리, 정확한 센서-베이스 변환(Sensor-to-Base Transformation)이 필수적이다. 라이다와 로봇 좌표계 사이의 외부 파라미터 보정(Extrinsic Calibration)이 잘못되면 스캔 정렬이 체계적으로 왜곡될 수 있다. 또한 로봇 운동과 시간적으로 일치하지 않는 타임스탬프(Timestamp)는 특히 빠른 병진이나 회전 중에 기하학적 오차처럼 보이는 문제를 발생시킬 수 있다.

레이저 스캔을 SLAM에 반영하는 빈도는 지도 반응성과 계산 비용 사이의 균형을 고려해야 한다. 로봇이 천천히 이동하거나 센서가 높은 주파수로 동작하는 경우 모든 스캔을 처리할 필요는 없다. GMapping 구현에서는 일반적으로 선형 이동 갱신 임계값(Linear Update Threshold)과 각도 갱신 임계값(Angular Update Threshold)을 사용하여 로봇이 충분한 거리를 이동한 경우에만 새로운 지도 갱신을 수행한다. 시간 임계값(Temporal Threshold)을 추가하여 로봇이 거의 정지한 경우에도 필요한 갱신을 보장할 수 있다.

GMapping은 벽, 복도, 방, 선반, 고정 설비와 같이 안정적인 기하 구조(Stable Geometric Structure)가 존재하는 환경에서 특히 효과적으로 동작한다. 이러한 구조는 강력한 스캔 정합 제약조건을 제공하고 서로 다른 파티클 궤적을 구별하는 데 도움을 준다. 반면 특징이 부족한 긴 복도, 넓은 개방 공간, 높은 대칭성을 가진 공간, 유리 구조물이 많은 환경, 이동하는 사람과 객체가 지배적인 장소에서는 레이저 관측이 약하거나 잘못된 위치추정 정보를 제공할 수 있어 성능이 저하될 수 있다.

명시적인 루프 폐쇄 제약조건(Loop-Closure Constraint)을 생성하고 이후 전역 그래프 최적화(Global Graph Optimization)를 수행하는 자세 그래프 SLAM(Pose-Graph SLAM)과 달리, GMapping은 파티클 궤적을 통해 과거의 불확실성을 표현한다. 로봇이 기존 영역을 다시 방문하면 스캔 우도(Scan Likelihood)는 새로운 관측과 기존 지도가 잘 정렬되는 파티클을 선호할 수 있다. 그러나 누적 오차가 지나치게 크거나 파티클 다양성이 부족하면 복구가 어려워져 장기간 및 대규모 지도작성에서 현대적인 그래프 기반 기법보다 불리할 수 있다.

구현 과정에서는 좌표계(Coordinate Frame) 사이의 일관성이 필요하다. 일반적인 이동 로봇은 전역 지도 좌표계(Global Map Frame), 지역적으로 연속적인 운동을 표현하는 오도메트리 좌표계(Odometry Frame), 로봇 베이스 좌표계(Robot Base Frame), 레이저 센서 좌표계(Laser Sensor Frame)를 구분한다. 이 좌표계 사이의 변환은 정확한 시간 정보와 함께 제공되어야 한다. SLAM 시스템은 지도 좌표계와 오도메트리 좌표계 사이의 관계를 추정하여 지역 오도메트리는 부드럽게 유지하면서 전역 위치 보정이 지도 내부의 로봇 위치를 수정하도록 한다.

ROS 기반 시스템(ROS-Based System)에서 GMapping은 역사적으로 레이저 스캔을 구독하고 변환 시스템(Transform System)을 통해 로봇 운동 정보를 획득하는 구성요소로 통합되어 왔다. 파티클 수, 지도 해상도, 갱신 거리, 스캔 거리, 정합 동작, 재샘플링 임계값, 운동 모델 잡음 등을 파라미터로 조절할 수 있다. 성공적인 적용을 위해서는 이러한 파라미터를 독립적으로 조정하기보다 하나의 통합 시스템으로 튜닝해야 한다. 한 요소의 변화가 파티클 분포, 스캔 정합, 지도 품질에 동시에 영향을 주기 때문이다.

실제 디버깅(Debugging)은 파티클 필터 파라미터를 변경하기 전에 원시 센싱(Raw Sensing)과 좌표 기하(Coordinate Geometry)를 검증하는 것에서 시작해야 한다. 레이저 스캔이 로봇과 정확하게 정렬되어야 하고, 오도메트리가 예상된 방향으로 연속적으로 움직여야 하며, 타임스탬프가 일관되어야 하고, 정적 구조물이 로봇 이동 중에도 안정적으로 유지되어야 한다. 이러한 기본 조건이 잘못되어 있다면 파티클 수를 늘리거나 재샘플링 임계값을 변경하더라도 계산량만 증가할 뿐 지도 오류의 근본 원인을 해결하기 어렵다.

자율이동로봇(Autonomous Mobile Robot, AMR)의 관점에서 GMapping은 완전한 파이프라인(Complete Pipeline)으로 동작하는 확률론적 SLAM(Probabilistic SLAM)을 직관적으로 보여주는 대표적인 사례이다. 오도메트리는 운동을 예측하고, 스캔 정합은 제안 분포를 개선하며, 파티클은 서로 다른 궤적을 표현하고, 측정 우도는 중요도를 부여한다. 적응형 재샘플링은 파티클 집합을 관리하고 점유 갱신(Occupancy Update)은 지도를 구축한다. 이러한 과정이 반복되면서 불확실한 센서 관측이 내비게이션에 사용할 수 있는 공간 표현으로 변환된다.

최신 그래프 최적화(Graph Optimization) 및 라이다-관성 기반 기법(LiDAR-Inertial Method)이 확장성과 장기적인 일관성 측면에서 더 높은 성능을 제공하는 경우가 많지만, GMapping은 확률론적 2D SLAM을 이해하고 구현하는 데 여전히 중요한 가치를 갖는다. GMapping의 구조는 베이지안 상태 추정(Bayesian State Estimation), 샘플링(Sampling), 중요도 가중(Importance Weighting), 재샘플링, 스캔 정합, 운동 모델링(Motion Modeling), 점유 지도작성(Occupancy Mapping)을 하나의 시스템 안에서 명확하게 보여준다. 이러한 기본 원리는 현대 SLAM 프레임워크가 서로 다른 수학적·계산적 구조를 사용하더라도 여전히 핵심적인 기반으로 활용된다.

## 03.03. Hector SLAM Scan Matching Based No Odometry [w/Code]

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

Hector SLAM은 휠 오도메트리(Wheel Odometry)에 명시적으로 의존하는 대신 고주파 레이저 스캔 정합(High-Frequency Laser Scan Matching)을 중심으로 설계된 2D 지도작성 및 위치추정(Mapping and Localization) 기법이다. 지속적으로 입력되는 2D 라이다(2D LiDAR) 스캔을 갱신되는 점유 격자 지도(Occupancy-Grid Map)와 정렬하여 로봇의 운동을 추정한다. 이러한 구조는 신뢰할 수 있는 휠 인코더(Wheel Encoder)를 사용할 수 없거나 통합하기 어렵고, 휠 슬립(Wheel Slip)이나 불안정한 지면 접촉의 영향을 크게 받는 경우에 특히 유용하다.

핵심 가정은 연속적인 레이저 관측(Laser Observation)에 로봇 자세 변화를 추정하기에 충분한 기하학적 정보가 포함되어 있다는 것이다. Hector SLAM은 휠 회전으로 운동을 우선 예측하는 대신 현재 스캔을 지도에 이미 표현된 구조와 가장 잘 정렬하는 평면 자세(Planar Pose) \\(x=[x,y,\\theta]\^T\\)를 탐색한다. 따라서 정확한 자세 추정은 스캔 주파수(Scan Frequency), 환경의 기하 구조(Environmental Geometry), 센서 품질, 연속 관측 사이의 운동 크기에 크게 의존한다.

레이저 측정값(Laser Measurement)은 극좌표 거리 관측(Polar Range Observation)에서 센서 좌표계(Sensor Coordinate Frame)의 직교좌표 종단점(Cartesian Endpoint)으로 변환된다. 후보 로봇 자세가 주어지면 이 점들을 지도 좌표계(Map Coordinate Frame)로 변환하고 점유 표현(Occupancy Representation)과 비교한다. 자세 추정은 관측된 장애물 점이 높은 점유 확률(Occupancy Probability)을 가진 위치에 배치되도록 하는 변환을 찾으며, 이를 통해 위치추정 문제를 평면 병진과 회전에 대한 연속 최적화(Continuous Optimization) 문제로 변환한다.

Hector SLAM의 핵심 구성요소 가운데 하나는 점유 격자 표현(Occupancy-Grid Representation)이다. 환경은 점유 상태의 추정값을 포함하는 셀(Cell)로 분할되고 레이저 관측을 통해 이 값이 지속적으로 갱신된다. 주로 이산 특징 추출(Discrete Feature Extraction)에 의존하는 방법과 달리 스캔 정합기는 다수의 레이저 종단점에 분산된 기하학적 정보를 직접 활용할 수 있다. 벽, 모서리, 기둥, 기계 설비 및 기타 안정적인 표면이 함께 로봇 운동 추정에 기여한다.

연속적인 스캔 정렬을 지원하기 위해 Hector SLAM은 이산 격자 셀 사이의 점유값에 대한 보간(Interpolation)을 사용한다. 변환된 레이저 종단점은 격자 셀의 중심에 정확하게 위치하는 경우가 드물기 때문에 가장 가까운 셀의 값만 평가하면 불연속적인 목적함수(Objective Function)가 생성된다. 이중선형 보간(Bilinear Interpolation)은 보다 부드러운 지도 응답을 생성하여 로봇 위치의 작은 변화도 스캔 정합 점수에 의미 있는 변화를 만들도록 하며, 기울기 기반 자세 최적화(Gradient-Based Pose Optimization)를 가능하게 한다.

스캔 정합 목적함수(Scan-Matching Objective)는 변환된 레이저 종단점이 지도의 점유 영역과 얼마나 잘 일치하는지를 측정한다. 초기 자세 추정값(Initial Pose Estimate)에서 시작하여 알고리즘은 정합 점수가 \\(x\\), \\(y\\), \\(\\theta\\)에 따라 어떻게 변화하는지를 계산한다. 이러한 미분값(Derivative)은 정렬 상태를 개선하기 위해 자세 추정값이 이동해야 하는 방향을 제공한다. 반복 최적화(Iterative Optimization)를 통해 스캔과 지역 지도(Local Map)가 충분히 일관된 상태에 도달할 때까지 자세를 정교화한다.

Hector SLAM은 일반적으로 가우스-뉴턴 방식 최적화(Gauss-Newton-Style Optimization)를 사용한다. 로봇 자세와 변환된 레이저 점 사이의 비선형 관계(Nonlinear Relationship)를 국소적으로 근사하여 세 개의 평면 자세 변수와 관련된 작은 시스템을 생성한다. 각각의 스캔 정합 과정에서 작은 자세 상태만 최적화하기 때문에 충분한 컴퓨팅 자원과 라이다 주파수가 확보되면 비교적 높은 갱신 주파수(Update Rate)로 효율적으로 동작할 수 있다.

휠 오도메트리를 사용하지 않더라도 적절한 초기 추정값은 여전히 중요하다. 일반적인 운용 조건에서는 고주파 관측 사이의 로봇 운동이 작다고 가정할 수 있기 때문에 이전에 추정된 자세가 다음 스캔 정합의 시작점으로 사용된다. 로봇이 스캔 사이에서 지나치게 멀리 이동하거나 빠르게 회전하면 초기 추정값이 수렴 영역(Convergence Region)을 벗어날 수 있으며, 이 경우 정합기가 잘못된 정렬 상태로 수렴할 수 있다.

Hector SLAM의 특징적인 요소 가운데 하나는 다중 해상도 지도 표현(Multi-Resolution Map Representation)이다. 서로 다른 공간 해상도를 가진 여러 점유 지도를 유지하여 지도 피라미드(Map Pyramid)를 구성한다. 스캔 정합은 유효 탐색 범위가 상대적으로 넓은 저해상도 단계에서 시작한 후 점차 높은 해상도로 이동할 수 있다. 이러한 거친 단계에서 정밀 단계로 진행하는 방식(Coarse-to-Fine Strategy)은 수렴 성능을 향상시키고 큰 규모의 정렬과 정확한 지역 보정을 함께 수행할 수 있도록 한다.

다중 해상도 구조는 스캔 정합이 지역적인 기하학적 모호성(Local Geometric Ambiguity)에 빠지는 문제를 완화하는 데 특히 유용하다. 낮은 해상도에서는 작은 지도 세부 요소가 억제되고 환경의 주요 구조가 최적화 방향을 결정한다. 이후 높은 해상도 단계에서 기하학적 세부 정보가 다시 반영되어 자세 정밀도를 향상시킨다. 이러한 접근법은 점진적으로 정교해지는 표현을 이용해 강건성과 정확도를 함께 향상시키는 컴퓨터 비전(Computer Vision)의 다중 스케일 최적화(Multi-Scale Optimization)와 유사하다.

휠 오도메트리 없이 동작한다는 것이 운동 가정(Motion Assumption)이나 외부 정보를 전혀 사용하지 않는다는 의미는 아니다. Hector SLAM은 관성 측정(Inertial Measurement)을 사용할 수 있는 경우, 특히 방향 정보(Orientation Information)를 활용하여 성능을 향상시킬 수 있다. 관성 측정 장치(Inertial Measurement Unit, IMU)는 자세나 빠른 회전 운동을 제한하는 데 도움을 주며 평면 휠 운동학으로 완벽하게 설명하기 어려운 플랫폼의 강건성을 향상시킬 수 있다. 그러나 핵심 평면 위치추정 메커니즘은 여전히 레이저-지도 스캔 정합(Laser-to-Map Scan Matching)이다.

필수적인 휠 오도메트리가 없다는 특성은 상당한 휠 슬립이 발생하는 로봇에서 장점을 제공한다. 기존의 인코더 기반 오도메트리(Encoder-Based Odometry)는 미끄러운 바닥, 느슨한 지면, 경사로 또는 스키드 스티어(Skid-Steer) 방식의 플랫폼에서 잘못된 운동 정보를 생성할 수 있다. Hector SLAM은 환경의 기하 구조로부터 변위를 추정하므로 휠 회전 오차가 자세 추정값에 직접 누적되지 않는다. 다만 이러한 장점은 레이저 관측 자체에 충분하고 안정적인 기하학적 제약조건이 존재하는 경우에만 유지된다.

특징이 부족한 환경(Feature-Poor Environment)은 근본적인 한계로 남는다. 예를 들어 길고 직선적인 복도에서는 레이저 측정이 횡방향 위치와 방향은 강하게 제한하지만 복도 진행 방향의 변위에 대해서는 제한적인 정보만 제공할 수 있다. 넓은 개방 공간에서도 신뢰성 있는 정렬을 위한 주변 표면이 부족할 수 있다. 반복적인 기하 구조(Repetitive Geometry)는 여러 개의 타당한 정렬 후보를 생성하여 잘못된 수렴이나 갑작스러운 위치추정 오차의 가능성을 증가시킨다.

동적 객체(Dynamic Object) 역시 스캔 정합에 영향을 미친다. Hector SLAM은 관측되는 대부분의 기하 구조가 정적인 환경(Static Environment)에 속한다고 가정하기 때문이다. 이동하는 사람, 차량, 문, 장비 또는 기타 일시적인 객체는 지속적인 지도와 일치하지 않는 레이저 반사값을 생성할 수 있다. 동적 관측이 하나의 스캔에서 지배적인 비중을 차지하면 최적화기가 잘못된 자세 방향으로 이동할 수 있다. 거리 필터링(Range Filtering), 환경 설계, 센서 배치 및 추가 센싱을 통해 이러한 문제를 완화할 수 있다.

라이다 특성(LiDAR Characteristics)은 성능에 큰 영향을 준다. 높은 스캔 주파수는 연속 관측 사이의 변위를 감소시켜 이전 자세가 다음 최적화의 유효한 초기값이 될 가능성을 높인다. 넓은 각도 범위(Wide Angular Coverage)와 충분한 측정거리는 더 많은 환경 제약조건을 제공한다. 정확한 센서 보정(Sensor Calibration)도 중요하며, 체계적인 각도 또는 거리 오차는 최적화에 사용되는 기하 구조를 왜곡하여 반복 관측 과정에서 일관되지 않은 지도 구조를 생성할 수 있다.

시간 동기화(Timing)는 구현 과정에서 또 하나의 핵심 요소이다. 레이저 측정은 하나의 순간에 동시에 획득되는 것이 아니라 일정한 스캔 시간 간격에 걸쳐 수집된다. 스캔이 획득되는 동안 로봇이 빠르게 움직이면 서로 다른 레이저 빔이 약간씩 다른 센서 자세에서 측정되어 운동 왜곡(Motion Distortion)이 발생한다. 이를 보상하지 않으면 생성된 스캔이 하나의 강체 변환(Rigid Transformation)으로 표현되지 않을 수 있으며, 특히 빠른 회전 상황에서 정합 정확도가 감소할 수 있다.

지도 해상도(Map Resolution)는 환경과 센서 특성에 따라 선택해야 한다. 높은 해상도는 좁은 구조와 정밀한 벽 경계를 표현할 수 있지만 메모리 소비량을 증가시키고 측정 잡음에 최적화가 더 민감해질 수 있다. 낮은 해상도 지도는 계산 요구량을 줄이고 지역적인 변화를 부드럽게 만들지만 기하학적 세부 정보를 잃게 된다. 다중 해상도 아키텍처는 여러 공간 규모를 함께 활용함으로써 수렴 강건성과 최종 정밀도 사이의 유용한 균형을 제공한다.

현대적인 자세 그래프 SLAM(Pose-Graph SLAM)과 달리 Hector SLAM은 명시적인 전역 루프 폐쇄 최적화(Global Loop-Closure Optimization)보다 지역 스캔-지도 정렬(Local Scan-to-Map Alignment)과 증분 지도작성(Incremental Mapping)에 주로 초점을 맞춘다. 과거 자세의 그래프를 생성한 후 루프 폐쇄 보정값을 전체 궤적에 분배하는 방식에 근본적으로 의존하지 않는다. 따라서 대규모 환경에서 누적된 오차는 로봇이 이전 위치로 돌아오더라도 완전히 보정하기 어려울 수 있다.

이러한 특성으로 인해 Hector SLAM은 전역적으로 최적화된 장기 일관성(Long-Term Consistency)보다 빠른 지역 지도작성(Local Mapping)이 중요한 응용 분야에 특히 적합하다. 소형 실내 로봇, 실험용 플랫폼, 휴대형 지도작성 시스템 또는 신뢰할 수 있는 휠 인코더가 없는 로봇 등이 대표적인 사례이다. 대규모 시설, 반복적인 장시간 임무 또는 강력한 루프 폐쇄 보정이 필요한 환경에서는 그래프 기반 SLAM이나 지역 스캔 정합과 전역 최적화를 결합한 시스템이 더 높은 확장성을 제공할 수 있다.

ROS 구현(ROS Implementation)에서는 휠 오도메트리가 없더라도 정확한 좌표 변환(Coordinate Transformation)이 필수적이다. 라이다 좌표계(LiDAR Frame)는 로봇 베이스 좌표계(Robot Base Frame)와 정확하게 연결되어야 하며 SLAM 시스템은 지도와 로봇 자세 사이의 일관된 관계를 제공해야 한다. 센서 타임스탬프, 좌표계 식별자(Frame Identifier), 레이저 각도 규약(Laser Angle Convention), 유효 거리 범위, 갱신 주파수 등을 스캔 정합 파라미터를 조정하기 전에 검증해야 한다.

따라서 실제 디버깅은 원시 레이저 스캔(Raw Laser Scan)과 로봇 좌표계로의 변환을 시각적으로 확인하는 것에서 시작해야 한다. 플랫폼이 움직이는 동안 정적인 벽은 안정적인 기하 구조를 유지해야 하며 지도작성을 시작하기 전에 연속 스캔이 적절하게 중첩되어야 한다. 스캔이 회전되거나 이동되어 보이고, 시간적으로 지연되거나 기하학적으로 왜곡된다면 최적화 파라미터를 조정하더라도 근본적인 보정 또는 동기화 오류를 안정적으로 해결할 수 없다.

자율이동로봇(Autonomous Mobile Robot, AMR)의 관점에서 Hector SLAM은 휠 변위를 주요 위치추정 정보로 요구하지 않고 환경 관측으로부터 직접 로봇 운동을 추정할 수 있다는 중요한 원리를 보여준다. 고주파 라이다 센싱(High-Rate LiDAR Sensing), 점유 격자 기울기(Occupancy-Grid Gradient), 반복적 스캔 정합, 다중 해상도 최적화가 결합되어 완전한 지역 SLAM 파이프라인(Local SLAM Pipeline)을 구성한다. 이러한 특성은 파티클 필터 및 그래프 기반 접근법과 비교할 수 있는 중요한 구조적 차이를 제공한다.

Hector SLAM의 보다 넓은 의미는 적절한 조건에서 강력한 환경 센싱(Environmental Sensing)이 신뢰성이 낮은 고유수용성 운동 추정(Proprioceptive Motion Estimation)을 대체할 수 있음을 보여준다는 데 있다. 성능은 센서 주파수, 기하학적 관측 가능성(Geometric Observability), 지도 표현, 보간, 최적화, 다중 스케일 처리(Multi-Scale Processing)의 상호작용을 통해 결정된다. 이러한 메커니즘을 이해하는 것은 현대 자율 로봇에서 사용되는 더욱 발전된 라이다 오도메트리(LiDAR Odometry)와 스캔 정합 시스템을 이해하기 위한 기반을 제공한다.

## 03.04. Cartographer 2D SLAM Configuration and Tuning [w/Code]

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

Cartographer 2D SLAM은 높은 주파수의 지역 자세 추정(Local Pose Estimation)과 전역적으로 일관된 궤적 최적화(Globally Consistent Trajectory Optimization)를 결합하도록 설계된 그래프 기반 지도작성 및 위치추정(Graph-Based Mapping and Localization) 프레임워크이다. 하나의 추정 메커니즘에 의존하지 않고 SLAM을 지역 궤적 생성(Local Trajectory Building)과 전역 자세 그래프 최적화(Global Pose-Graph Optimization)로 분리한다. 이러한 아키텍처를 통해 이동 로봇은 반응성이 높은 실시간 위치추정을 유지하면서 루프 폐쇄 제약조건(Loop-Closure Constraint)을 이용하여 누적된 드리프트(Drift)를 주기적으로 보정할 수 있다.

지역 SLAM(Local SLAM) 구성요소는 2D 라이다(2D LiDAR) 또는 호환 가능한 거리 센서(Range Sensor)의 관측값을 처리하고 주변 지도 데이터를 기준으로 연속적인 로봇 자세를 추정한다. 입력되는 스캔은 누적되고 필터링된 후 지역 서브맵(Local Submap)에 정렬된다. 이를 통해 생성된 자세 추정값은 부드러운 단기 운동 추적(Short-Term Motion Tracking)을 제공하며, 완성된 서브맵과 선택된 궤적 노드(Trajectory Node)는 이후 전역 최적화 문제의 구성요소가 된다.

Cartographer는 하나의 거대한 전역 점유 격자(Global Occupancy Grid)를 지속적으로 수정하는 대신 서브맵(Submap)을 사용하여 환경을 표현한다. 각각의 서브맵은 로봇이 특정 지역을 이동하면서 수집한 제한된 수의 거리 관측(Range Observation)으로 구성된다. 새로운 스캔은 활성 서브맵(Active Submap)에 삽입되며 충분한 관측이 누적되면 이전 서브맵을 완료하여 이후 제약조건 생성(Constraint Generation)과 전역 최적화(Global Optimization)에 사용할 수 있도록 보존한다.

서브맵을 사용하면 계산 및 추정 측면에서 모두 장점을 얻을 수 있다. 스캔 정합(Scan Matching)은 지속적으로 커지는 전체 전역 지도가 아니라 제한된 지역 표현만을 대상으로 수행할 수 있다. 동시에 이미 완성된 서브맵은 이후의 궤적 노드와 비교할 수 있는 안정적인 기준을 제공한다. 이러한 구조는 이전에 방문했던 영역을 탐지하고 공간적으로 연관된 관측 사이에 루프 폐쇄 제약조건을 추가하기 위한 자연스러운 기반을 형성한다.

스캔 정합 이전에 Cartographer는 거리 데이터 전처리(Range-Data Preprocessing)를 수행하여 잡음과 계산량을 감소시킨다. 설정된 최소 및 최대 거리 범위를 벗어나는 측정값은 제거하거나 별도로 처리할 수 있으며, 누적된 스캔에는 복셀 필터링(Voxel Filtering)을 적용할 수 있다. 적절한 설정값은 센서 제조사가 제시하는 이론적인 최대 측정거리만 사용하는 것이 아니라 라이다의 신뢰 가능한 동작 범위, 환경 규모, 예상 장애물 밀도, 로봇 속도를 반영해야 한다.

지역 자세 추정은 일반적으로 자세 예측(Pose Prediction)과 스캔 정합을 결합한다. 로봇 구성에 따라 예측 과정에는 오도메트리(Odometry), 관성 측정(Inertial Measurement), 또는 이전에 추정된 운동을 사용할 수 있다. 실시간 상관 스캔 정합기(Real-Time Correlative Scan Matcher)는 예측된 자세 주변을 탐색하여 강건한 초기 정렬을 생성할 수 있으며, 이후 비선형 스캔 정합(Nonlinear Scan Matching)이 활성 서브맵 및 사용 가능한 운동 사전정보(Motion Prior)와의 일치도를 최적화하여 병진과 회전을 정교화한다.

실시간 상관 스캔 정합기는 초기 자세 예측에 불확실성이 존재할 때 강건성을 향상시키지만 탐색 과정에서 계산 비용이 증가한다. 선형 또는 각도 탐색 범위(Linear or Angular Search Window)를 확대하면 더 큰 예측 오차에서 복구할 수 있지만 평가해야 하는 후보 자세의 수가 크게 증가한다. 따라서 특히 지도작성과 인지(Perception), 경로 계획(Planning), 제어(Control)를 동시에 수행해야 하는 임베디드 컴퓨터(Embedded Computer)에서는 강건성과 지연시간(Latency) 사이의 균형을 고려하여 설정해야 한다.

비선형 스캔 정합은 점유 공간 일치도(Occupied-Space Agreement), 병진 편차(Translation Deviation), 회전 편차(Rotation Deviation)에 관련된 가중 목적함수(Weighted Objective)를 이용하여 자세를 정교화한다. 이러한 가중치는 해가 지도 기하 구조(Map Geometry)와 예측 자세 가운데 어느 쪽을 얼마나 강하게 따를지를 결정한다. 지나치게 강한 운동 사전정보는 부정확한 오도메트리의 보정을 방해할 수 있으며, 반대로 사전정보가 너무 약하면 모호한 환경 기하 구조가 해를 잘못된 정렬 방향으로 이동시킬 수 있다.

거리 데이터 삽입(Range Data Insertion)은 지역 서브맵이 어떻게 구축되는지를 결정한다. 레이저 광선(Laser Ray)은 빔을 따라 자유 공간(Free Space)에 대한 정보를 제공하고 유효한 종단점 부근에서는 점유 공간(Occupied Space)에 대한 정보를 제공한다. 적중 및 비적중 확률(Hit and Miss Probability)을 제어하는 파라미터는 점유 증거(Occupancy Evidence)가 얼마나 빠르게 누적되는지를 결정한다. 공격적인 갱신은 선명한 지도를 빠르게 생성하지만 일시적인 측정값까지 강화할 수 있으며, 보수적인 갱신은 반복 관측을 요구하는 대신 잡음이 많은 환경에서 더욱 안정적인 지도를 생성할 수 있다.

운동 필터링(Motion Filtering)은 거의 동일한 관측의 삽입을 방지하여 불필요한 계산을 줄일 수 있다. Cartographer는 이전에 수용된 관측 이후의 경과 시간, 병진 거리, 회전량을 비교하여 새로운 공간 정보가 거의 없는 갱신을 제거할 수 있다. 임계값은 로봇 속도와 센서 주파수에 따라 선택해야 한다. 지나치게 강한 필터링은 유용한 기하 정보를 제거할 수 있으며, 반대로 약한 필터링은 중복 노드를 생성하여 최적화 비용을 증가시킬 수 있다.

전역 SLAM(Global SLAM)은 궤적 노드, 서브맵 자세(Submap Pose), 그리고 이들을 연결하는 제약조건으로 구성된 자세 그래프(Pose Graph)를 생성한다. 지역 제약조건(Local Constraint)은 관측과 지역 지도작성에 직접 사용된 주변 서브맵을 연결한다. 시스템이 궤적 노드와 이전에 구축된 서브맵 사이에서 타당한 정합 후보를 발견하면 추가적인 제약조건을 생성할 수 있다. 이러한 비지역 관계(Nonlocal Relationship)는 루프 폐쇄와 장기 드리프트 보정(Long-Term Drift Correction)을 가능하게 한다.

제약조건 생성은 Cartographer 튜닝(Tuning)에서 가장 큰 영향을 미치는 요소 가운데 하나이다. 후보 제약조건을 지나치게 넓은 범위에서 자주 탐색하면 CPU 사용량이 증가하고 잘못된 정합(False Match)이 발생할 가능성도 커진다. 반대로 지나치게 보수적으로 탐색하면 유효한 루프 폐쇄를 발견하지 못할 수 있다. 따라서 샘플링 비율(Sampling Ratio), 정합 점수(Matching Score), 탐색 범위(Search Range), 제약조건 생성기(Constraint Builder)의 동작과 관련된 파라미터는 계산 확장성과 전역 지도 일관성(Global Map Consistency)에 동시에 영향을 준다.

루프 폐쇄 후보(Loop-Closure Candidate)는 일반적으로 기존 서브맵에 대한 스캔 정합을 통해 평가된다. 후보 정합은 제약조건으로 채택되기 전에 충분한 일치도를 확보해야 한다. 임계값이 너무 낮으면 잘못된 정합이 수용되어 전체 궤적을 왜곡할 수 있으며, 지나치게 높은 임계값은 관측 시점 변화(Viewpoint Change), 센서 잡음, 환경 변화가 존재할 때 정상적인 루프 폐쇄까지 거부할 수 있다. 따라서 임계값은 실제 운용 환경을 대표하는 데이터를 이용해 검증해야 한다.

제약조건이 수집되면 Cartographer는 궤적 노드와 서브맵 자세를 대상으로 희소 비선형 최적화 문제(Sparse Nonlinear Optimization Problem)를 주기적으로 해결한다. 최적화기는 지역 스캔 정합 관계, 오도메트리 정보, 관성 제약조건(Inertial Constraint), 루프 폐쇄를 동시에 만족시키도록 동작한다. 유효한 루프 폐쇄가 추가되면 보정값이 전체 궤적에 분산되어 현재 로봇 자세만 갑자기 이동시키는 대신 전역적인 일관성을 향상시킨다.

전역 최적화의 실행 주기(Global Optimization Frequency)는 반응성과 계산 요구량 모두에 영향을 준다. 지나치게 자주 최적화를 수행하면 새로운 제약조건이 거의 추가되지 않은 상황에서도 CPU 부하만 증가할 수 있다. 반대로 너무 드물게 수행하면 전역적인 불일치가 오랫동안 유지되고 루프 폐쇄 보정도 지연된다. 실제 설정에서는 궤적 길이, 프로세서 성능, 예상되는 루프 발생 빈도, 대규모 시설에서 지도작성을 지속적으로 수행하는지 여부 등을 고려해야 한다.

오도메트리는 장기적인 드리프트가 존재하더라도 지역적으로 부드러운 운동 추정값을 제공한다면 Cartographer의 성능을 크게 향상시킬 수 있다. SLAM 시스템은 오도메트리를 단기 운동 제약조건(Short-Term Motion Constraint)으로 사용하고 레이저 관측과 루프 폐쇄를 통해 누적 오차를 보정할 수 있다. 그러나 잘못된 좌표 규약(Coordinate Convention), 스케일 오차(Scale Error), 불연속적인 운동 정보, 과도한 인코더 잡음은 스캔 정합을 저하시킬 수 있으므로 설정 가중치를 높이기 전에 오도메트리 품질을 독립적으로 검증해야 한다.

관성 측정 장치(Inertial Measurement Unit, IMU)는 특히 빠르게 회전하거나 휠 기반 요각(Yaw) 추정이 신뢰하기 어려운 플랫폼에서 유용한 회전 정보를 제공할 수 있다. 정확한 IMU 방향(Orientation), 타임스탬프 동기화(Timestamp Synchronization), 좌표 변환(Frame Transformation)이 필수적이다. 평면 2D SLAM이 최종적으로 \\(x\\), \\(y\\), 요각을 중심으로 운동을 추정하더라도 잘못된 관성 데이터는 일관되지 않은 운동 사전정보를 생성하여 위치추정 품질을 떨어뜨릴 수 있다.

좌표계 설정(Coordinate-Frame Configuration)은 신뢰성 높은 시스템 적용의 기본 조건이다. 지도 좌표계(Map Frame), 발행되는 추적 좌표계(Published Tracking Frame), 로봇 베이스 좌표계(Robot Base Frame), 오도메트리 좌표계(Odometry Frame), 개별 센서 좌표계(Sensor Frame)는 일관된 변환 트리(Transformation Tree)를 구성해야 한다. Cartographer는 센서 관측을 정확한 위치에 배치하기 위해 시간에 따라 변화하는 좌표 변환을 사용한다. 누락된 변환, 잘못된 좌표 방향, 지연된 변환 발행은 SLAM 파라미터가 적절하더라도 스캔 정합 실패처럼 나타날 수 있다.

로봇의 속도가 증가할수록 센서 동기화(Sensor Synchronization)의 중요성도 증가한다. 타임스탬프 또는 시스템 시계가 일치하지 않으면 라이다 스캔, 오도메트리 갱신, IMU 측정값이 서로 다른 실제 시점의 로봇 상태를 나타낼 수 있다. 이러한 시간 오차는 센서 사이의 공간적인 불일치처럼 나타난다. 따라서 스캔 정합 가중치를 조정하기 전에 전체 데이터 파이프라인에서 시계 동기화(Clock Synchronization), 타임스탬프 의미, 좌표 변환 가용성, 센서 지연시간을 검증해야 한다.

지도 해상도(Map Resolution)는 기하학적 세부 표현, 메모리 사용량, 스캔 정합 동작에 영향을 준다. 높은 해상도는 벽과 좁은 구조를 정확하게 표현하지만 지도 크기를 증가시키고 정렬 과정이 잡음에 민감해질 수 있다. 낮은 해상도는 계산 비용을 줄이지만 가까운 구조물을 하나로 병합할 수 있다. 따라서 라이다 정확도, 환경에서 표현해야 하는 최소 구조물 크기, 로봇 크기, 후속 내비게이션(Downstream Navigation)에 요구되는 정밀도를 고려하여 해상도를 결정해야 한다.

체계적인 튜닝 과정(Disciplined Tuning Process)은 여러 값을 동시에 변경하는 대신 관찰되는 실패 현상(Failure Mode)에 따라 파라미터 그룹을 단계적으로 조정한다. 먼저 정확한 센서 기하 구조, 필터링, 운동 예측, 스캔 정합 설정을 이용하여 지역 스캔 정렬(Local Scan Alignment)을 안정화해야 한다. 이후 서브맵 구성을 확인하고 제약조건 생성과 전역 최적화를 순차적으로 검토한다. 이러한 단계적 접근은 지역 추정 오류와 전역 루프 폐쇄 문제를 구분하기 쉽게 만든다.

기록된 센서 데이터셋(Recorded Sensor Dataset)은 파라미터를 변경하면서 동일한 궤적을 반복 재생할 수 있기 때문에 설정 작업에 특히 유용하다. 개발자는 제어된 조건에서 궤적의 부드러움, 벽 정렬, 루프 폐쇄 동작, CPU 사용량, 최적화 지연시간을 비교할 수 있다. 대표적인 데이터셋에는 정상적인 운용뿐만 아니라 좁은 복도, 개방 공간, 빠른 회전, 동적 객체, 긴 루프와 같은 어려운 상황도 포함되어야 한다.

자율이동로봇(Autonomous Mobile Robot, AMR)에서 성공적인 Cartographer 튜닝은 하나의 보편적인 최적 파라미터를 찾는 문제가 아니라 시스템 수준의 공학적 작업(System-Level Engineering Task)이다. 센서 품질, 로봇 운동학(Robot Kinematics), 프로세서 성능, 환경의 기하 구조, 운용 속도, 내비게이션 정확도 요구조건이 서로 강하게 영향을 미친다. 따라서 실제 플랫폼과 실제 환경에서 설정을 검증하면서 전체 자율주행 스택(Autonomy Stack)을 위한 충분한 계산 여유를 유지해야 한다.

Cartographer 2D SLAM은 지역 스캔 정합(Local Scan Matching)과 전역 그래프 최적화(Global Graph Optimization)를 확장 가능한 하나의 아키텍처 안에서 어떻게 결합할 수 있는지를 보여준다. 지역 궤적 생성은 반응성이 높은 자세 추정값을 제공하고, 서브맵은 거리 관측을 구조화하며, 제약조건 생성은 공간적 관계를 발견하고, 자세 그래프 최적화는 전체 궤적에 걸쳐 보정량을 분배한다. 적절한 설정은 이러한 구성요소를 조화시켜 실시간 위치추정과 장기적인 지도 일관성이 서로 충돌하지 않고 상호 보완하도록 만든다.

## 03.05. SLAM Toolbox Lifelong Mapping for AMR [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

SLAM Toolbox는 이동 로봇(Mobile Robot)의 지도작성(Mapping), 위치추정(Localization), 지도 개선(Map Refinement), 장기 운용(Long-Term Operation)을 위해 설계된 ROS 기반 2D SLAM 프레임워크이다. 특히 평생 지도작성(Lifelong Mapping) 기능은 시간이 지나면서 변화하는 시설에서 반복적으로 운용되는 자율이동로봇(Autonomous Mobile Robot, AMR)에 적합하다. 지도작성을 일회성 시운전 작업으로 취급하지 않고 지도작성 세션(Mapping Session)을 보존하여 이후 임무에서 공간 지식(Spatial Knowledge)을 지속적으로 개선하거나 확장할 수 있다.

평생 지도작성에서 근본적인 과제는 실제 운용 환경이 완전히 정적인 상태로 유지되는 경우가 거의 없다는 점이다. 선반의 위치가 변경되거나 생산 장비가 재배치될 수 있으며, 복도 구조가 변경되고 이전에는 접근할 수 없었던 영역이 새롭게 개방될 수도 있다. 동시에 사람, 카트, 팔레트, 차량과 같은 일시적인 객체(Temporary Object)가 자동으로 영구 지도(Permanent Map)를 변경해서는 안 된다. 따라서 유용한 평생 SLAM 시스템은 지속적인 공간 변화(Persistent Spatial Change)와 일시적인 관측(Transient Observation)을 구분해야 한다.

SLAM Toolbox는 로봇 자세와 관련 레이저 관측이 공간 제약조건(Spatial Constraint)의 그래프를 구성하는 자세 그래프 기반 아키텍처(Pose-Graph-Oriented Architecture)를 따른다. 연속된 자세는 지역 운동(Local Motion) 및 스캔 정합(Scan Matching) 관계를 통해 연결되며, 루프 폐쇄(Loop Closure)는 서로 다른 시점에 동일한 위치에서 획득한 관측을 연결할 수 있다. 최적화(Optimization)는 이러한 제약조건이 전역적으로 일관되도록 그래프를 조정하여 누적된 궤적 드리프트(Trajectory Drift)를 감소시킨다.

레이저 스캔 정합(Laser Scan Matching)은 관측과 변화하는 지도 사이의 핵심적인 기하학적 관계를 제공한다. 입력되는 2D 라이다(2D LiDAR) 데이터는 이전에 저장된 공간 정보와 비교되어 로봇 자세를 추정하고 제약조건을 생성한다. 오도메트리(Odometry)는 유용한 단기 운동 정보를 제공하고 스캔 정합은 환경의 기하 구조를 기준으로 드리프트를 보정한다. 따라서 안정적인 성능을 위해서는 일관된 라이다 보정(LiDAR Calibration), 안정적인 오도메트리, 좌표 변환(Coordinate Transform), 동기화된 타임스탬프(Timestamp)가 필요하다.

SLAM Toolbox의 주요 장점 가운데 하나는 최종 점유 격자 이미지(Occupancy-Grid Image)만 보존하는 것이 아니라 지도작성 정보를 직렬화(Serialization)하여 저장할 수 있다는 점이다. 일반적인 지도 이미지는 최종적인 공간 표현만을 저장하며 이를 생성한 추정 구조(Estimation Structure)의 상당 부분을 잃는다. 직렬화된 SLAM 데이터(Serialized SLAM Data)는 지도작성을 재개하고 자세 그래프를 다시 최적화하는 데 필요한 정보를 보존할 수 있으므로 기존 환경을 처음부터 다시 구축하지 않고 이전 지도작성 상태에서 계속 작업할 수 있다.

저장된 지도작성 세션이 복원되면 로봇은 탐색을 진행하면서 새로운 관측과 제약조건을 계속 추가할 수 있다. 새롭게 접근한 영역은 기존 지도를 확장할 수 있으며, 다시 방문한 영역은 현재 관측을 이전에 작성된 기하 구조와 정렬할 기회를 제공한다. 이러한 동작은 시설이 점진적으로 확장되거나 기존 공간 정보를 모두 폐기하지 않고 주기적인 재지도작성(Remapping)을 수행해야 하는 환경에 적합하다.

평생 지도작성은 단순한 지도 확장보다 더 어려운 문제를 포함한다. 시스템이 환경 변화(Environmental Change)를 처리해야 하기 때문이다. 벽이나 고정 설비가 실제로 이동했다면 새로운 관측은 과거 측정값과 충돌할 수 있다. 반면 일시적으로 배치된 팔레트나 주차된 카트는 반드시 영구적인 지도 구조가 되어야 하는 것은 아니다. 따라서 운용 정책(Operational Policy), 반복 관측, 필터링, 통제된 지도 갱신 절차(Controlled Map-Update Procedure)는 SLAM 알고리즘을 보완하는 중요한 요소가 된다.

루프 폐쇄는 긴 궤적과 반복적인 방문에서 일관성을 유지하는 데 여전히 핵심적이다. 현재 관측이 이전에 지도화된 위치와 대응하면 루프 폐쇄 제약조건이 자세 그래프에서 시간적으로 멀리 떨어진 부분을 연결할 수 있다. 이후 그래프 최적화(Graph Optimization)는 보정량을 관련된 여러 자세에 분산시킨다. 특히 긴 통로와 반복적인 구조 패턴이 존재하여 지각적 모호성(Perceptual Ambiguity)이 발생할 수 있는 창고와 공장에서는 신뢰성 높은 루프 탐지가 중요하다.

잘못된 루프 폐쇄(False Loop Closure)는 기하학적으로 유사하지만 실제로는 서로 다른 위치를 잘못 연결하여 평생 지도를 심각하게 왜곡할 수 있다. 산업 환경에는 반복되는 랙(Rack), 문, 기둥, 생산 셀(Production Cell)이 많이 존재한다. 따라서 스캔 정합 임계값(Scan-Matching Threshold)과 루프 탐색 파라미터는 모호한 정합을 거부할 수 있을 만큼 보수적으로 설정하면서도 중간 수준의 환경 변화가 존재할 때 실제 재방문을 인식할 수 있도록 조정해야 한다.

자세 그래프 최적화(Pose-Graph Optimization)는 지역 제약조건과 루프 폐쇄 제약조건의 집합을 전역적으로 일관된 궤적 추정(Global Trajectory Estimate)으로 변환한다. 직접적인 관계가 있는 자세들만 연결되기 때문에 생성되는 최적화 문제는 희소 구조(Sparse Structure)를 가지며 효율적으로 해결할 수 있다. 그러나 평생 운용에서는 자세와 제약조건의 수가 지속적으로 증가할 수 있으므로 그래프 크기, 메모리 소비량, 최적화 주기, 계산 지연시간(Computational Latency)이 중요한 공학적 고려사항이 된다.

SLAM Toolbox는 적용 요구조건에 따라 동기식 처리(Synchronous Processing) 또는 비동기식 처리(Asynchronous Processing) 구성으로 운용할 수 있다. 동기식 방식은 센서 정보의 결정론적 처리(Deterministic Processing)를 단순화할 수 있으며, 비동기식 방식은 센서 획득과 최적화 작업이 서로 다른 속도로 수행되는 실시간 시스템에서 유용할 수 있다. 적절한 구성은 라이다 주파수, 로봇 속도, 사용 가능한 CPU 자원, 전체 자율주행 스택(Autonomy Stack)의 계산 요구량에 따라 결정된다.

기존 지도에 대한 위치추정과 지도를 지속적으로 수정하는 작업은 개념적으로 구분해야 한다. 일반적인 생산 운용 중 AMR에는 제한 없는 지도작성보다 안정적인 위치추정이 우선적으로 필요할 수 있다. 모든 관측이 장기적인 공간 지식을 수정하도록 허용하면 일시적인 객체가 지도에 포함될 수 있다. 따라서 실제 아키텍처에서는 운용 요구조건에 따라 일상적인 위치추정(Routine Localization), 통제된 지도 갱신(Controlled Map Update), 전용 재지도작성(Dedicated Remapping)을 분리할 수 있다.

AMR 적용에서는 이러한 구분을 통해 효과적인 지도 수명주기(Map Lifecycle)를 구성할 수 있다. 초기 시운전 임무(Commissioning Mission)를 통해 검증된 기준 지도(Baseline Map)를 생성하고, 일반 운용에서는 해당 지도를 이용해 위치추정을 수행하며, 지속적인 환경 변화가 확인된 경우 승인된 유지보수 세션(Authorized Maintenance Session)을 통해 특정 영역을 갱신할 수 있다. 이러한 관리 방식은 평생 지도작성이 통제되지 않은 연속적인 지도 변경으로 변하는 것을 방지하고 산업 운용 과정에서 공간 변화를 추적할 수 있도록 한다.

지도 품질은 격자 해상도(Grid Resolution)와 센서 정확도의 관계에 크게 영향을 받는다. 정밀한 점유 격자는 세부적인 벽, 좁은 통로, 장비 경계를 보존할 수 있지만 더 많은 메모리를 소비하고 작은 정렬 오차를 두드러지게 만들 수 있다. 낮은 해상도의 격자는 잡음에 더 강하고 계산 비용이 낮지만 내비게이션에 필요한 세부 구조를 제거할 수 있다. 따라서 해상도는 라이다 정확도, 로봇 풋프린트(Robot Footprint), 안전 여유 공간(Clearance Requirement), 환경 구조물의 크기를 고려하여 결정해야 한다.

스캔 정합과 루프 폐쇄가 보정 기능을 제공하더라도 오도메트리 품질(Odometry Quality)은 여전히 중요하다. 부드러운 지역 오도메트리는 연속 관측 사이에서 최적화기에 유용한 예측값을 제공하고 스캔 정렬에 필요한 탐색 범위를 감소시킨다. 휠 슬립, 인코더 스케일 오차(Encoder Scale Error), 불연속적인 측정, 잘못된 운동학 파라미터(Kinematic Parameter)는 정합의 난이도를 높일 수 있다. 따라서 SLAM 파라미터를 적극적으로 조정하기 전에 오도메트리 하위 시스템을 독립적으로 보정하고 평가해야 한다.

좌표계 일관성(Coordinate-Frame Consistency)도 기본적인 요구조건이다. 지도(Map), 오도메트리, 로봇 베이스(Robot Base), 레이저 좌표계(Laser Frame)는 정확한 변환을 통해 연결되어야 하며 이러한 변환은 적절한 센서 타임스탬프에서 사용할 수 있어야 한다. 잘못된 라이다 장착 변환(LiDAR Mounting Transform)은 체계적인 지도 왜곡을 발생시킬 수 있으며, 지연된 좌표 변환은 실제로 존재하지 않는 운동 오차처럼 나타날 수 있다. 스캔 정합이나 최적화 파라미터를 변경하기 전에 이러한 기반 인프라 문제를 제거해야 한다.

장기 운용에서는 지도 저장(Map Storage)과 버전 관리(Version Management)도 중요하다. 직렬화된 SLAM 상태는 가치 있는 운용 데이터이며 일회성 임시 파일이 아니라 통제되는 산출물(Controlled Artifact)로 관리해야 한다. 기준 지도, 갱신 지도, 검증 상태(Validation Status), 생성 날짜, 로봇 구성, 롤백 지점(Rollback Point)을 관리하면 문제가 있는 지도 갱신이 검증된 기존 운용 지도를 영구적으로 대체하는 것을 방지할 수 있다.

다중 로봇 시설(Multi-Robot Facility)에서는 추가적인 아키텍처 고려사항이 발생한다. 여러 AMR이 동일한 물리적 환경에서 동작할 수 있지만 모든 로봇이 하나의 공유 지도를 독립적으로 수정하도록 허용하면 서로 일치하지 않는 관측과 통제되지 않은 변경이 발생할 수 있다. 보다 안전한 전략은 권위 있는 지도 수명주기(Authoritative Map Lifecycle)를 정의하고 로봇들이 관측 또는 지도작성 세션을 제공하되 지도 검증, 병합(Merging), 배포(Release)는 시스템 수준에서 통제하는 것이다.

동적 환경(Dynamic Environment)에서는 점유 관측을 신중하게 해석해야 한다. 창고 통로에 일시적으로 팔레트나 작업자가 존재할 수 있지만 공간의 구조적 경계는 변하지 않을 수 있다. 개별 스캔을 필터링하는 것도 유용하지만 시간에 따른 지속성(Persistence)은 구조적인 변화를 판단하는 더욱 강력한 지표가 될 수 있다. 따라서 장기 지도작성에서는 내비게이션에 사용하는 단기 장애물 정보(Short-Lived Obstacle Information)와 전역 SLAM 지도에 포함되는 지속적인 기하 구조(Persistent Geometry)를 분리하는 것이 유용하다.

기록된 데이터셋(Recorded Dataset)과 반복 가능한 주행 경로(Repeatable Route)는 평생 지도작성 동작을 튜닝하는 효과적인 기반을 제공한다. 동일한 센서 데이터를 반복 재생하면서 스캔 정합 임계값, 루프 폐쇄 설정, 그래프 최적화 파라미터, 지도 해상도를 변경할 수 있다. 평가는 궤적 일관성, 벽 정렬, 루프 폐쇄 정확성, CPU 사용량, 메모리 증가량, 직렬화된 상태에서 지도작성을 재개하는 능력을 함께 고려해야 한다.

생산용 AMR에서는 장애 복구(Failure Recovery)가 특히 중요하다. 센서 가림, 급격한 위치 변화, 환경 변경, 잘못된 초기화 등으로 로봇이 위치추정을 상실할 수 있다. 시스템은 신뢰도가 낮아진 위치추정 상태를 감지해야 하며 이를 인식하지 못한 채 지도를 계속 손상시켜서는 안 된다. 복구 방법에는 기존 기하 구조에 대한 재위치추정(Relocalization), 운영자 지원 초기화(Operator-Assisted Initialization), 알려진 영역으로의 복귀, 신뢰도가 회복될 때까지 장기 지도 갱신을 일시적으로 중지하는 방법 등이 포함될 수 있다.

평생 운용은 수개월 또는 수년에 걸쳐 지속될 수 있기 때문에 계산 확장성(Computational Scalability)도 고려해야 한다. 모든 관측을 항상 최대 세부 수준으로 유지하는 것이 반드시 바람직한 것은 아니다. 실제 시스템에서는 노드 선택(Node Selection), 그래프 유지관리(Graph Maintenance), 예약된 최적화(Scheduled Optimization), 세션 보관(Archived Session), 통제된 재지도작성 등을 활용하여 과거 데이터가 사용 가능한 자원을 과도하게 소비하는 것을 방지할 수 있다. 목표는 중복된 측정값을 무제한으로 축적하는 것이 아니라 지속 가능한 공간 지식을 유지하는 것이다.

AMR 플릿(AMR Fleet)의 관점에서 SLAM Toolbox의 장기적인 가치는 단순히 점유 지도를 생성하는 것 이상에 있다. 공간 지식을 저장하고 복원하며 확장하고 최적화하고 검증하고 버전 관리할 수 있는 유지관리 가능한 자산(Maintainable Asset)으로 취급할 수 있는 프레임워크를 제공한다. 이는 로봇이 지속적으로 운용되는 동안 시설 배치, 장비, 저장 패턴, 접근 가능 영역이 점진적으로 변화하는 산업 환경에서 특히 중요하다.

강건한 평생 지도작성 전략(Robust Lifelong Mapping Strategy)은 궁극적으로 SLAM 알고리즘과 운용 거버넌스(Operational Governance)를 결합한다. 스캔 정합과 자세 그래프 최적화는 기하학적 일관성을 제공하고, 직렬화는 지도작성 상태를 보존하며, 위치추정 모드는 일반 운용을 지원하고, 통제된 갱신은 지속적인 환경 변화에 대응한다. 이러한 메커니즘을 결합하면 로봇의 추정이나 환경이 영구적으로 변하지 않는다고 가정하지 않으면서도 AMR이 반복적인 임무에 걸쳐 유용한 공간 지식을 지속적으로 유지할 수 있다.

## 03.06. Loop Closure Detection 2D Scan Matching [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

루프 폐쇄 탐지(Loop Closure Detection)는 로봇이 이전에 방문했던 위치로 다시 돌아왔음을 인식하고, 현재 자세(Current Pose)와 과거 궤적(Historical Trajectory) 사이에 신뢰할 수 있는 공간 제약조건(Spatial Constraint)을 설정하는 과정이다. 2D 라이다 SLAM(2D LiDAR SLAM)에서는 증분 오도메트리(Incremental Odometry)와 지역 스캔 정합(Local Scan Matching)에서 필연적으로 드리프트(Drift)가 누적되므로 이 기능이 매우 중요하다. 유효한 루프 폐쇄는 SLAM 시스템이 이러한 누적 오차를 보정하고 전역 일관성(Global Consistency)을 회복할 수 있도록 한다.

어려운 점은 단순히 서로 유사한 레이저 스캔(Laser Scan)을 인식하는 것이 아니라 두 관측이 실제로 동일한 물리적 위치에 대응하는지를 판단하는 것이다. 복도, 선반, 문, 기둥, 반복적인 산업 구조물은 서로 다른 위치에서도 기하학적으로 유사한 측정값을 생성할 수 있다. 따라서 새로운 제약조건이 전역 지도(Global Map)에 영향을 미치도록 허용하기 전에 후보 생성(Candidate Generation)과 기하학적 검증(Geometric Verification)을 모두 수행해야 한다.

2D 레이저 스캔은 주변의 기하 구조를 각도에 따라 배열된 일련의 거리 측정값(Range Measurement)으로 표현한다. 정합을 수행하기 전에 유효하지 않은 반사값, 유용한 거리 범위를 벗어난 측정값, 명백한 센서 이상값(Sensor Artifact)을 제거할 수 있다. 알고리즘에 따라 스캔을 직교좌표 점(Cartesian Point), 선 특징(Line Feature), 점유 표현(Occupancy Representation), 기술자(Descriptor) 또는 이전에 관측한 위치와의 비교를 용이하게 하는 다른 압축 구조로 변환할 수 있다.

후보 생성은 계산 비용이 높은 기하학적 정합을 수행해야 하는 과거 스캔의 수를 줄이는 역할을 한다. 드리프트가 크지 않은 경우 현재 궤적으로부터 예측된 공간적 근접성(Spatial Proximity)을 이용해 후보를 생성할 수 있으며, 외형과 유사한 스캔 기술자(Scan Descriptor)를 이용하면 추정 위치가 멀리 떨어져 있어도 기하학적으로 유사한 장소를 검색할 수 있다. 강건한 SLAM 시스템은 일반적으로 하나의 검색 방법에만 의존하지 않고 여러 단서를 결합한다.

스캔 기술자는 관측된 기하 구조를 효율적으로 검색할 수 있는 형태로 요약한다. 각도별 거리 패턴(Angular Range Pattern), 방사형 점유 분포(Radial Occupancy Distribution), 히스토그램(Histogram), 스펙트럼 특성(Spectral Property), 또는 추출된 기하 특징 사이의 관계 등을 표현할 수 있다. 기술자 정합(Descriptor Matching)은 일반적으로 그 자체로 루프 폐쇄를 확정하지 않으며, 보다 정밀한 기하학적 검증을 수행할 가치가 있는 과거 관측을 식별하는 역할을 한다.

후보가 선택되면 스캔 정합(Scan Matching)을 이용하여 현재 스캔과 과거 스캔 또는 서브맵(Submap) 사이의 상대 변환(Relative Transformation)을 추정한다. 2D 환경에서 이 변환은 \\(x\\) 및 \\(y\\) 방향의 병진(Translation)과 회전 \\(\\theta\\)로 구성된다. 정합기는 두 관측 사이의 기하학적 일관성(Geometric Consistency)을 최대화하거나 정렬 오차(Alignment Error)를 최소화하는 SE(2) 변환을 탐색한다.

이러한 검증 단계에는 여러 가지 정합 방식(Matching Formulation)을 사용할 수 있다. 점대점(Point-to-Point) 방식은 대응점 사이의 거리를 최소화하며, 점대선(Point-to-Line) 방식은 벽과 같은 국부적인 평면 구조를 활용한다. 상관 스캔 정합(Correlative Scan Matching)은 점유 표현에 대해 후보 변환을 평가하고, 우도장 방식(Likelihood-Field Method)은 변환된 종단점이 지도의 점유 영역에 얼마나 가까운지를 기준으로 점수를 계산한다. 각 방법은 속도, 포착 범위(Capture Range), 강건성 측면에서 서로 다른 특성을 갖는다.

반복 최근접점(Iterative Closest Point, ICP)은 기하학적 스캔 정렬을 직관적으로 보여주는 대표적인 방법이다. 두 관측의 점 사이에서 대응 관계를 추정하고, 불일치를 줄이는 변환을 계산한 후 이 과정을 반복한다. ICP는 초기값이 충분히 정확한 경우 높은 정밀도의 지역 정렬을 제공할 수 있지만, 스캔 중첩이 부족하거나 반복 구조로 인해 대응 관계가 모호하면 잘못된 국소 최솟값(Local Minimum)으로 수렴할 수 있다.

상관 스캔 정합은 순수한 지역 최적화(Local Optimization)보다 넓은 병진 및 회전 영역을 탐색할 수 있기 때문에 루프 폐쇄에 유용하다. 후보 자세(Candidate Pose)를 이산화된 지도(Discretized Map) 또는 확률 격자(Probability Grid)에 대해 평가하여 정합 점수를 계산한다. 단점은 탐색 범위, 각도 범위, 해상도, 후보 수가 증가할수록 계산 비용이 커진다는 것이다. 거친 단계에서 정밀 단계로 진행하는 탐색(Coarse-to-Fine Search)을 사용하면 이러한 계산 부담을 줄일 수 있다.

루프 후보는 하나의 정합 점수만으로 평가해서는 안 된다. 유용한 검증 기준에는 스캔 중첩량(Scan Overlap), 잔여 정렬 오차(Residual Alignment Error), 변환 크기(Transformation Magnitude), 지역 기하 구조와의 일관성, 추정 불확실성(Estimated Uncertainty) 등이 포함된다. 수치적으로 높은 점수를 얻었더라도 비현실적인 변환을 요구하는 후보는 잘못된 정합일 수 있으므로 기하학적 증거와 운동 및 지도 문맥(Map Context)을 함께 검증해야 한다.

추정된 상대 변환에는 불확실성 모델(Uncertainty Model)도 필요하다. 서로 직교하는 여러 벽에 의해 제약되는 정합은 병진과 회전에 대해 강한 정보를 제공할 수 있지만, 긴 직선 복도 내부의 정렬은 횡방향 변위와 방향은 강하게 제한하면서 복도 진행 방향의 위치에는 약한 정보를 제공할 수 있다. 따라서 루프 제약조건을 자세 그래프(Pose Graph)에 추가할 때 공분산(Covariance) 또는 정보 행렬(Information Matrix)은 이러한 비등방성 관측 가능성(Anisotropic Observability)을 반영해야 한다.

반복적인 환경(Repetitive Environment)은 2D 루프 폐쇄에서 가장 어려운 조건 가운데 하나이다. 창고의 서로 다른 두 통로에 거의 동일한 랙이 배치될 수 있으며, 사무실 복도에는 동일한 형태의 문과 벽 구조가 반복될 수 있다. 이러한 경우 실제로는 서로 다른 위치임에도 지역 스캔 정합기가 매우 높은 정렬 결과를 생성할 수 있다. 더 넓은 공간 문맥(Spatial Context), 연속적 일관성(Sequential Consistency), 추가 센서 또는 여러 인접 관측을 이용하면 이러한 지각적 에일리어싱(Perceptual Aliasing)을 감소시킬 수 있다.

연속적 일관성은 실제 재방문이 일반적으로 여러 개의 연속 관측에 걸쳐 지속된다는 특성을 활용한다. 현재 로봇 궤적이 과거의 특정 영역과 일치한다면 로봇이 계속 이동할 때 주변 스캔들도 서로 호환되는 변환을 생성해야 한다. 주변 관측의 지지를 받지 못하는 단일 후보는 신뢰도가 낮다고 판단할 수 있다. 이러한 시간적 추론(Temporal Reasoning)은 잘못된 양성 루프 폐쇄(False Positive Loop Closure)를 상당히 줄일 수 있다.

동적 객체(Dynamic Object)는 또 다른 정합 오류의 원인이 된다. 사람, 지게차, 카트, 팔레트, 주차된 차량, 이동 가능한 장비 등은 서로 다른 방문 시점에 다른 위치에 존재할 수 있다. 강건한 정합은 모든 레이저 반사값이 일치하도록 요구하기보다 지속적인 환경 구조(Persistent Environmental Structure)를 우선적으로 활용해야 한다. 이상치 제거(Outlier Rejection), 점유 확률, 강건 손실 함수(Robust Loss Function), 거리 필터링, 여러 스캔의 집계를 통해 일시적인 객체에 대한 민감도를 줄일 수 있다.

정확한 루프 폐쇄는 일반적으로 궤적 시간상 멀리 떨어져 있는 자세들 또는 자세와 서브맵을 연결하는 비지역 제약조건(Nonlocal Constraint)으로 자세 그래프에 추가된다. 이후 백엔드 최적화기(Back-End Optimizer)는 오도메트리, 지역 스캔 정합 및 다른 측정값과 함께 이 제약조건을 만족하도록 궤적을 조정한다. 보정량은 그래프 전체에 분산되어 현재 자세에서 인위적인 불연속을 발생시키지 않으면서 누적 드리프트를 감소시킨다.

잘못된 루프 폐쇄는 그래프 최적화가 수용된 제약조건에 의미 있는 정보가 포함되어 있다고 가정하기 때문에 특히 위험하다. 높은 가중치를 가진 잘못된 제약조건은 복도를 서로 겹치게 만들거나 지도 일부를 회전시키고 실제로 분리된 공간을 하나로 붕괴시킬 수 있다. 따라서 일반적으로 탐지되는 루프의 단순한 수를 최대화하는 것보다 정밀도(Precision)를 높이는 것이 중요하다. 불확실한 루프 폐쇄를 잘못 수용하는 것보다 놓치는 것이 더 안전한 경우가 많다.

강건 그래프 최적화(Robust Graph Optimization)는 추가적인 방어 수단을 제공하지만 프런트엔드 검증(Front-End Verification)을 대체해서는 안 된다. 강건 손실 함수는 비정상적으로 큰 잔차를 갖는 제약조건의 영향을 줄일 수 있으며, 전환 가능 제약(Switchable Constraint) 또는 일관성 인식 메커니즘(Consistency-Aware Mechanism)은 호환되지 않는 엣지를 억제할 수 있다. 그러나 내부적으로 그럴듯한 잘못된 루프는 초기 잔차가 작을 수 있으므로 신뢰성 높은 시스템은 보수적인 탐지와 백엔드 강건성을 함께 사용한다.

루프 폐쇄의 탐색 빈도(Loop Closure Frequency)는 계산 확장성(Computational Scalability)에도 영향을 준다. 새로운 모든 스캔을 모든 과거 스캔과 비교하면 임무 시간이 증가함에 따라 계산량도 계속 증가한다. 후보 인덱싱(Candidate Indexing), 기술자 데이터베이스(Descriptor Database), 서브맵 표현, 공간 검색 구조(Spatial Search Structure), 샘플링 정책을 이용하여 비교 횟수를 제한할 수 있다. 장기간 운용되는 AMR에서는 저장된 지도 데이터가 증가하더라도 루프 탐지가 실용적인 수준을 유지하도록 이러한 메커니즘이 필요하다.

임계값 튜닝(Threshold Tuning)은 실제 환경을 대표하는 양성 및 음성 사례(Positive and Negative Example)를 사용하여 수행해야 한다. 실제 재방문 사례에는 관측 시점 변화, 중간 수준의 환경 변화, 서로 다른 교통 상황을 포함해야 하며, 음성 사례에는 의도적으로 외형적 또는 기하학적으로 반복되는 영역을 포함해야 한다. 이를 통해 하나의 편리한 경로에서만 성능을 최적화하는 대신 누락된 루프와 잘못된 수용(False Acceptance) 사이의 균형을 평가하여 정합 임계값을 선택할 수 있다.

기록된 데이터셋(Recorded Dataset)은 후보 탐색 범위, 기술자 임계값, 스캔 정합 파라미터, 수용 기준(Acceptance Criterion)을 변경하면서 동일한 궤적을 반복적으로 처리할 수 있기 때문에 특히 유용하다. 평가는 탐지된 루프의 개수뿐만 아니라 상대 자세 정확도(Relative-Pose Accuracy), 잘못된 양성 비율(False-Positive Rate), 전역 궤적 오차(Global Trajectory Error), 지도 일관성, CPU 부하, 메모리 소비량, 최적화 지연시간을 함께 고려해야 한다.

센서 보정(Sensor Calibration)과 동기화(Synchronization)는 신뢰성 높은 루프 탐지의 선행 조건이다. 체계적인 라이다 장착 오차는 로봇이 회전할 때 관측되는 기하 구조를 변화시키며, 시간 오차는 스캔과 로봇 운동 사이의 관계를 왜곡한다. 이러한 문제는 실제 루프의 정합 점수를 낮추거나 잘못된 정렬을 생성할 수 있다. 따라서 루프 폐쇄 임계값을 잘못된 센서 기하 구조나 타임스탬프를 보상하기 위한 수단으로 사용해서는 안 된다.

자율이동로봇(Autonomous Mobile Robot, AMR)에서 루프 폐쇄는 단순한 이진 패턴 인식(Binary Pattern Recognition) 이벤트가 아니라 신뢰도 기반 프로세스(Confidence-Controlled Process)로 이해해야 한다. 후보 검색(Candidate Retrieval)은 로봇이 다시 방문했을 가능성이 있는 위치를 제안하고, 기하학적 스캔 정합은 두 관측이 어떻게 정렬되는지를 추정하며, 검증 과정은 해당 관계의 신뢰성을 판단하고, 자세 그래프 최적화는 수용된 보정값을 전체 궤적에 적용한다.

전체적인 목표는 루프 폐쇄의 수를 최대화하는 것이 아니라 전역 일관성을 유지하는 데 필요한 충분한 수의 신뢰할 수 있는 장거리 제약조건(Long-Range Constraint)을 추가하는 것이다. 효과적인 2D 루프 폐쇄는 효율적인 후보 탐색, 정확한 SE(2) 스캔 정합, 관측 가능성을 고려한 불확실성(Observability-Aware Uncertainty), 모호한 기하 구조의 거부, 시간적 일관성, 강건 그래프 최적화를 결합한다. 이러한 메커니즘은 반복적으로 관측되는 환경 정보를 장기 SLAM(Long-Term SLAM)을 위한 신뢰성 높은 보정 정보로 변환한다.

## 03.07. Map Post Processing Cleaning and Annotation [w/Code]

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

원시 2D SLAM 지도(Raw 2D SLAM Map)는 완성된 내비게이션 자산(Navigation Asset)이 아니라 추정 결과물(Estimation Product)이다. 위치추정(Localization)과 스캔 정합(Scan Matching)이 정상적으로 수행되더라도 점유 격자(Occupancy Grid)에는 고립된 잡음, 중복된 벽 경계, 일시적인 장애물, 불확실한 셀 또는 센서 반사와 궤적 오차로 인해 발생한 인공물(Artifact)이 포함될 수 있다. 지도 후처리(Map Post-Processing)는 이러한 확률적 결과물을 자율이동로봇(Autonomous Mobile Robot, AMR)의 내비게이션에 적합한 보다 깨끗하고 통제된 표현으로 변환한다.

후처리는 기본적인 SLAM 품질을 평가한 이후에 시작해야 한다. 지도 정리(Cleaning)를 통해 심각한 궤적 변형(Trajectory Deformation), 잘못된 루프 폐쇄(False Loop Closure), 체계적인 라이다 보정 오류(LiDAR Calibration Error), 심각한 시간 동기화 문제를 안정적으로 수정할 수는 없다. 벽이 전체적으로 잘못 정렬되거나 반복 구조가 넓은 영역에서 이중으로 나타난다면 결과 점유 지도를 외형적으로 수정하는 대신 SLAM 파이프라인을 수정하고 지도를 다시 생성하는 것이 적절하다.

점유 격자는 환경을 점유 공간(Occupied Space), 자유 공간(Free Space), 미확인 공간(Unknown Space)을 나타내는 값이 할당된 이산 셀(Discrete Cell)로 표현한다. 내비게이션용으로 내보낼 때 이러한 확률값은 일반적으로 지도 해상도(Map Resolution), 원점(Origin), 점유 임계값(Occupancy Threshold)을 설명하는 메타데이터(Metadata)와 함께 회색조 이미지(Grayscale Image) 값으로 변환된다. 이미지 크기, 해상도, 방향 또는 원점을 관련 메타데이터의 변경 없이 수정하면 실제 물리 좌표가 무효화될 수 있으므로 후처리 과정에서 이러한 기하학적 관계를 유지해야 한다.

지도 정리는 일반적으로 지속적인 구조물에 대응하지 않는 고립된 점유 셀(Isolated Occupied Cell)과 작은 군집(Cluster)을 제거하는 것에서 시작한다. 이러한 인공물은 먼지, 이동 객체(Moving Object), 거리 측정 잡음(Range Noise), 유리 반사, 일시적인 스캔 정합 오류 등으로 발생할 수 있다. 연결 요소 분석(Connected-Component Analysis), 이웃 필터링(Neighborhood Filtering), 형태학적 연산(Morphological Operation), 또는 신중하게 통제된 수동 편집을 이용하면 벽, 기둥 및 기타 내비게이션에 필요한 구조를 유지하면서 이러한 영역을 제거할 수 있다.

일시적인 객체(Temporary Object)는 지도작성 임무 중에는 존재했더라도 반드시 영구 지도(Permanent Map)의 일부가 되어야 하는 것은 아니므로 특별한 주의가 필요하다. 팔레트, 카트, 의자, 차량, 열린 문, 사람 등이 점유 흔적을 남길 수 있다. 작은 객체를 자동으로 제거하면 실제 고정 인프라(Fixed Infrastructure)까지 삭제할 수 있으므로, 이러한 객체의 제거 여부는 운용 환경에 대한 지식이나 반복적인 관측을 기반으로 결정해야 한다.

벽 정리(Wall Cleanup)는 일반적으로 연속적이고 기하학적으로 안정적인 경계(Geometrically Stable Boundary)를 만드는 것을 목표로 한다. 작은 틈은 비용지도 팽창(Costmap Inflation)이나 경로 계획 과정에서 실제로는 폐쇄되어야 할 영역을 통과 가능한 공간으로 인식하게 만들 수 있으며, 지나치게 두껍거나 중복된 벽은 이동 가능한 공간을 불필요하게 감소시킬 수 있다. 형태학적 닫힘(Morphological Closing), 선 적합(Line Fitting), 윤곽선 처리(Contour Processing), 수동 보정을 통해 경계를 개선할 수 있지만 지나친 정규화(Regularization)는 위치추정에 필요한 실제 기하 특징을 제거할 수 있다.

미확인 공간을 자동으로 자유 공간으로 변환해서는 안 된다. 미확인 셀(Unknown Cell)은 지도작성 센서가 해당 영역에 대해 충분한 정보를 제공하지 않았음을 의미하는 반면, 자유 셀(Free Cell)은 실제로 관측되어 이동 가능한 공간임을 나타낸다. 탐색되지 않은 영역을 자유 공간으로 처리하면 전역 경로 계획기(Global Planner)가 검증되지 않은 영역을 통과하는 경로를 생성할 수 있다. 따라서 미확인 공간과 자유 공간의 구분은 로봇의 내비게이션 및 안전 정책(Safety Policy)과 일관되게 유지되어야 한다.

지도 자르기(Map Cropping)는 지도 환경 주변의 사용하지 않는 넓은 여백을 제거하여 저장 공간과 처리 부하를 줄일 수 있다. 그러나 자르기는 이미지 좌표계(Image Coordinate System)를 변경하므로 지도 원점 또는 관련 메타데이터도 함께 조정해야 한다. 픽셀 좌표(Pixel Coordinate)가 위치추정 및 내비게이션 구성요소에서 사용하는 미터 단위 좌표(Metric Coordinate)와 더 이상 일치하지 않는다면 지도 자체는 시각적으로 올바르게 보여도 공간적으로는 잘못된 상태가 될 수 있다.

이미지 편집 과정에서도 해상도를 통제해야 한다. 일반적인 그래픽 소프트웨어를 사용하여 지도의 크기를 변경하면 픽셀과 실제 거리 사이의 관계가 변할 수 있으며, 점유 의미를 갖지 않는 보간된 회색조 값(Interpolated Grayscale Value)이 생성될 수 있다. 지도 해상도를 변경해야 한다면 일반적인 이미지 크기 조절의 결과로 우연히 변경하는 것이 아니라 점유 정보를 고려한 재표본화(Occupancy-Aware Resampling)와 메타데이터 갱신을 통해 의도적으로 수행해야 한다.

주석(Annotation)은 점유 정보만으로 충분히 표현할 수 없는 의미적 또는 운용 정보를 추가한다. 내비게이션 지도에는 적재 구역(Loading Zone), 도킹 스테이션(Docking Station), 엘리베이터(Elevator), 충전 위치(Charging Location), 제한 구역(Restricted Area), 검사 지점(Inspection Point), 교차로(Intersection), 명명된 생산 셀(Named Production Cell) 등이 필요할 수 있다. 이러한 주석이 실제 물리적 장애물을 의도적으로 나타내는 경우가 아니라면 점유 격자에 직접 그려 넣기보다 구조화된 데이터 계층(Structured Data Layer)으로 관리하는 것이 일반적으로 적절하다.

의미 주석(Semantic Annotation)은 점(Point), 다각형(Polygon), 선분(Line Segment), 영역(Region), 명명된 좌표계(Named Coordinate Frame) 등으로 표현할 수 있다. 도킹 스테이션은 위치와 방향을 포함하는 목표 자세(Target Pose)로 저장할 수 있으며 제한 구역은 다각형으로 표현할 수 있다. 선호 이동 통로(Preferred Travel Corridor)는 경로나 차선 구조(Lane Structure)로 표현할 수 있다. 의미 정보와 점유 정보를 분리하면 기하학적 지도를 변경하지 않고도 운용 규칙을 갱신할 수 있다.

진입 금지 구역(No-Go Zone)은 산업용 AMR 운용에서 특히 중요하다. 물리적으로 개방된 공간이라도 작업자 작업구역, 위험 장비, 보안 경계(Security Boundary), 바닥 하중 제한(Floor-Loading Limit), 공정 요구조건 때문에 로봇의 진입이 금지될 수 있다. 이러한 제한을 전용 내비게이션 계층(Navigation Layer)에 표현하면 물리적 점유 지도는 기하학적으로 정확하게 유지하면서 경로 계획 정책이 로봇의 이동 허용 영역을 독립적으로 정의할 수 있다.

속도 구역(Speed Zone)은 운용 주석의 또 다른 사례이다. AMR은 보행자 횡단구역, 사각지대 코너(Blind Corner), 출입문, 작업 셀, 도킹 영역 근처에서 해당 공간을 물리적으로 주행할 수 있더라도 속도를 줄여야 할 수 있다. 속도 제한을 공간 메타데이터(Spatial Metadata)로 저장하면 기본 SLAM 표현을 변경하지 않고도 경로 계획기나 제어기(Controller)가 상황에 따른 운동 제한을 적용할 수 있다.

일방통행 차선(One-Way Lane)과 선호 경로(Preferred Route)는 여러 로봇이 운용되는 창고 및 공장에서 교통 흐름을 개선할 수 있다. 점유 격자는 물리적으로 이동 가능한 영역을 나타내지만 방향성을 가진 교통 규칙은 표현하지 못한다. 그래프 또는 차선 주석(Graph or Lane Annotation)을 추가하면 허용 이동 방향, 선호 경로, 교차로 우선순위(Intersection Priority), 경로 비용(Route Cost)을 표현할 수 있다. 이러한 계층은 단일 로봇 내비게이션에서 플릿 수준 조정(Fleet-Level Coordination)으로 확장될수록 더욱 중요해진다.

도킹 및 충전 주석(Docking and Charging Annotation)은 일반적인 영역 라벨보다 높은 정밀도를 요구한다. 충전 위치에는 일반적으로 기준 자세(Reference Pose), 접근 방향(Approach Direction), 정렬 허용오차(Alignment Tolerance), 필요한 경우 단계적 접근 경로(Staged Approach Path)가 포함되어야 한다. 이러한 주석이 지도에서 파생되었다면 모든 지도 편집 이후에도 해당 좌표가 유효해야 한다. 따라서 지도 원점, 회전 또는 해상도가 변경되면 저장된 모든 운용 랜드마크(Operational Landmark)를 체계적으로 다시 검증해야 한다.

지도 후처리에서는 위치추정 요구조건(Localization Requirement)도 고려해야 한다. 시각적으로 불필요해 보이는 특징이라도 라이다 위치추정에 유용한 기하 정보를 제공할 수 있다. 작은 기둥, 벽의 오목한 부분(Recess), 벽면의 불규칙성을 제거하면 지도는 더 깔끔하게 보일 수 있지만 스캔 기반 위치추정(Scan-Based Localization)에 필요한 식별성을 감소시킬 수 있다. 따라서 지도 정리는 실제 센서 관측과 일치하지 않는 이상화된 평면도(Idealized Floor Plan)를 만드는 것이 아니라 내비게이션 사용성을 개선하는 방향으로 수행해야 한다.

유용한 원칙은 위치추정 지도(Localization Map)와 운용 내비게이션 계층(Operational Navigation Layer)을 구분하는 것이다. 위치추정 지도에는 로봇이 실제로 관측할 수 있는 안정적인 물리 구조를 유지해야 하며, 내비게이션 계층에는 팽창 장애물(Inflated Obstacle), 제한 구역, 속도 제한, 의미 구역(Semantic Zone), 플릿 규칙(Fleet Rule)을 포함할 수 있다. 이러한 분리는 운용 정책이 위치추정에 사용되는 기하학적 기준을 손상시키는 것을 방지한다.

편집 이후의 검증(Validation)은 기하학적 검사와 운용 검사를 모두 포함해야 한다. 처리된 지도에 기록된 레이저 스캔을 중첩하여 벽과 고정 구조물이 계속 정확하게 정렬되는지 확인할 수 있다. 알려진 랜드마크 사이의 거리를 이용하여 스케일(Scale)을 검증하고 선택된 기준 좌표를 이용하여 원점과 방향을 확인할 수 있다. 이후 내비게이션 테스트를 수행하여 경로 계획기가 장애물, 제한 구역 및 주석된 운용 영역을 올바르게 반영하는지 확인해야 한다.

시뮬레이션(Simulation)과 기록 데이터 재생(Recorded-Data Replay)은 실제 배포 전에 처리된 지도를 효율적으로 시험할 수 있는 방법이다. 이전에 기록한 로봇 궤적을 정리된 지도에 대해 다시 위치추정하여 원본 지도와 자세 안정성 및 스캔 정렬 성능을 비교할 수 있다. 또한 실제 로봇을 잘못된 지도에 즉시 노출하지 않고도 좁은 통로, 도킹 접근, 경로 제한, 경계 조건(Boundary Condition) 등의 경로 계획 시나리오를 시험할 수 있다.

후처리를 통해 수동 또는 의미적 수정이 추가되기 시작하면 지도 버전 관리(Map Version Control)가 필수적이다. 배포되는 각각의 지도에는 식별 가능한 버전, 생성 출처(Creation Source), 해상도, 좌표 기준(Coordinate Reference), 편집 이력(Editing History), 검증 상태, 호환 가능한 주석 집합(Annotation Set)이 포함되어야 한다. 원본 SLAM 결과도 보관하여 이후 변경 내용을 추적하고 잘못된 편집이 발생했을 때 전체 시설을 다시 지도작성하지 않고 이전 상태로 복원할 수 있어야 한다.

시간에 따라 변화하는 시설에서는 지도 유지관리(Map Maintenance)를 통제된 갱신 절차에 따라 수행해야 한다. 작은 운용 변경은 주석만 갱신하면 충분할 수 있지만 영구적인 구조 변경(Permanent Structural Change)이 발생하면 지역 재지도작성(Local Remapping) 또는 SLAM 지도의 재생성이 필요할 수 있다. 이러한 경우를 구분하면 불필요한 재지도작성을 방지하고 복잡한 수동 편집을 통해 큰 기하학적 변화를 억지로 기존 지도에 반영하는 문제도 피할 수 있다.

다중 로봇 시스템(Multi-Robot System)에서는 특히 엄격한 지도 일관성(Map Consistency)이 필요하다. 동일한 운용 환경을 사용하는 모든 AMR은 지도 해상도, 원점, 좌표계, 제한 구역, 의미 랜드마크를 동일하게 해석해야 한다. 오래된 지도 또는 주석 버전을 사용하는 로봇은 나머지 플릿과 다른 경로를 계획할 수 있다. 따라서 지도 배포(Map Distribution)는 단순한 파일 복사가 아니라 관리되는 구성 릴리스(Managed Configuration Release)로 취급해야 한다.

품질 보증(Quality Assurance)에는 연결되지 않은 자유 공간 영역, 벽의 비정상적인 개구부, 고립된 장애물 군집, 잘못된 회색조 값, 일관되지 않은 메타데이터, 지도 경계 밖의 주석 좌표, 서로 중첩되는 정책 구역(Policy Zone) 등을 탐지하는 자동 검사가 포함될 수 있다. 자동 검증(Automated Validation)이 시각적 검사를 대체할 수는 없지만 로봇 운용용 지도를 승인하기 전에 일반적인 편집 오류를 탐지하는 데 효과적이다.

따라서 최종 내비게이션 자산은 하나의 이미지가 아니라 서로 연계된 지도 패키지(Coordinated Map Package)로 이해하는 것이 적절하다. 여기에는 점유 격자, 메타데이터, 의미 랜드마크, 진입 금지 다각형(No-Go Polygon), 속도 구역, 경로 정보, 도킹 자세, 버전 식별자(Version Identifier), 검증 기록(Validation Record) 등이 포함될 수 있다. 모든 구성요소는 하나의 일관된 좌표 기준을 공유하고 함께 배포되어 위치추정, 경로 계획, 제어 및 플릿 관리가 환경을 동일하게 해석하도록 해야 한다.

효과적인 지도 후처리는 추정된 기하 구조(Estimated Geometry)와 운용 지식(Operational Knowledge)의 차이를 유지한다. 지도 정리는 근본적인 SLAM 오류를 감추지 않으면서 인공물을 제거하고, 기하학적 편집은 미터 단위의 공간적 일관성(Metric Consistency)을 유지하며, 주석은 점유 정보를 불필요하게 변경하지 않고 구조화된 의미를 추가한다. 체계적인 검증과 버전 관리를 통해 원시 2D SLAM 결과를 장기간 AMR 운용에 사용할 수 있는 신뢰성 높고 유지관리 가능한 내비게이션 자산으로 변환할 수 있다.

## 03.08. Multi Floor and Multi Zone 2D Map Management [w/Code]

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

다층 및 다중 구역 2D 지도 관리(Multi-Floor and Multi-Zone 2D Map Management)는 기존 SLAM을 하나의 연속적인 점유 격자(Occupancy Grid) 범위 이상으로 확장하는 개념이다. 대형 건물, 공장, 병원, 창고, 캠퍼스, 물류 시설은 하나의 평면 지도만으로 편리하게 표현하기 어려운 여러 층이나 운용 구역을 포함하는 경우가 많다. 따라서 실용적인 자율이동로봇(Autonomous Mobile Robot, AMR) 시스템은 여러 지역 지도(Local Map)와 이들을 연결하는 공간적·운용적 관계를 함께 관리해야 한다.

하나의 층(Floor)은 일반적으로 독립적인 2D 미터 단위 지도(2D Metric Map)로 표현한다. 평면 SLAM은 로봇의 운동이 주로 \\(x\\), \\(y\\), 요각(Yaw)에서 발생한다고 가정하기 때문이다. 서로 다른 층은 수평면에 투영했을 때 동일한 위치에 중첩될 수 있지만 실제로는 높이가 다른 별도의 공간이다. 층별 지도를 독립적으로 유지하면 이러한 중첩으로 인해 잘못된 기하학적 관계가 형성되는 것을 방지하고 각 층에서 검증된 점유 표현(Occupancy Representation)을 사용할 수 있다.

구역(Zone)은 하나의 층 내부에서 추가적인 공간 분할 수준을 제공한다. 대형 창고는 입고(Receiving), 저장(Storage), 생산(Production), 검사(Inspection), 충전(Charging), 출하(Shipping) 구역으로 구분할 수 있으며, 병원은 공용 복도, 출입 제한 임상 구역, 엘리베이터, 서비스 구역 등으로 구분할 수 있다. 구역 경계(Zone Boundary)는 물리적인 벽뿐만 아니라 기하 구조, 내비게이션 정책, 교통 관리, 위치추정 품질 또는 관리 책임을 기준으로 정의할 수 있다.

각 지도에는 명시적인 식별자(Map Identifier)와 좌표 기준(Coordinate Reference)이 있어야 한다. 메타데이터(Metadata)에는 일반적으로 지도 해상도(Map Resolution), 원점(Origin), 방향(Orientation), 층 또는 구역 식별자, 버전(Version), 생성 출처(Creation Source), 검증 상태(Validation State)가 포함된다. 이러한 속성을 통해 내비게이션 소프트웨어는 현재 활성화된 공간 표현을 판단하고 서로 다른 원점이나 스케일을 가진 다른 지도의 좌표가 잘못 해석되는 것을 방지할 수 있다.

계층적 지도 모델(Hierarchical Map Model)은 대규모 시설을 표현하기 위한 유용한 추상화 방법이다. 가장 상위 수준에서는 건물 또는 사이트(Site)를 표현하고, 그다음 수준에서는 층을 식별하며, 하위 수준에는 구역 또는 지역 지도를 배치한다. 이러한 영역 사이의 연결 관계는 전이(Transition)로 별도로 표현한다. 이를 통해 경로 계획은 먼저 어떤 영역들을 통과해야 하는지 결정한 후 선택된 각 2D 지도 내부에서 세부적인 미터 단위 경로 계획(Metric Planning)을 수행할 수 있다.

전이는 서로 분리된 지도들을 실제 운용에서 사용할 수 있도록 연결하기 때문에 매우 중요하다. 전이는 엘리베이터(Elevator), 경사로(Ramp), 출입문(Doorway), 에어록(Airlock), 이송 스테이션(Transfer Station), 통제 통로(Controlled Passage) 등을 나타낼 수 있다. 각각의 전이에는 진입 자세(Entry Pose)와 출구 자세(Exit Pose), 허용 이동 방향, 로봇 호환성(Robot Compatibility), 운용 조건, 그리고 양쪽 좌표계 사이의 관계가 정의되어야 한다.

엘리베이터 전이는 단순한 기하학적 연결 이상의 절차를 요구한다. 로봇은 대기 자세(Waiting Pose)로 접근하고 엘리베이터를 호출하며 도착 여부를 확인한 후 진입하고 안전한 위치에 정렬해야 한다. 이후 목적 층을 선택하거나 요청하고 문이 열렸는지 확인한 후 사전에 정의된 자세로 빠져나가야 한다. 엘리베이터 이동 중에는 일반적인 2D 위치추정이 불안정해질 수 있으므로 내비게이션 시스템은 층별 위치추정 문맥(Localization Context)의 전환을 명시적으로 관리해야 한다.

로봇이 엘리베이터에서 빠져나올 때 이전 층의 지도를 사용한 위치추정을 그대로 계속해서는 안 된다. 시스템은 목적 층 지도(Destination Map)를 활성화하고 알려진 출구 자세(Exit Pose) 주변에서 로봇의 초기 위치를 설정해야 한다. 이후 라이다 관측을 이용하여 목적 층의 기하 구조를 기준으로 자세를 정교화할 수 있다. 정상적인 자율 내비게이션을 재개하기 전에 위치 재추정(Relocalization)의 성공 여부를 신뢰도 검사를 통해 확인하여 잘못된 층 가설이 경로 실행에 전달되는 것을 방지해야 한다.

경사로는 로봇의 높이가 변하는 동안에도 물리적인 이동이 연속적으로 유지된다는 점에서 엘리베이터와 다르다. 순수한 2D 지도에서는 경사로를 층별 지도 사이의 전이로 표현하거나, 기하 구조와 위치추정 요구조건에 따라 하나의 확장된 내비게이션 영역(Extended Navigation Region)의 일부로 표현할 수 있다. 선택한 표현 방식은 중복되는 수평 좌표를 방지하고 진입 지점과 출구 지점 사이의 관계를 명확하게 유지해야 한다.

다중 구역 운용(Multi-Zone Operation)이 항상 점유 지도를 전환해야 한다는 의미는 아니다. 하나의 기하학적 층 지도(Geometric Floor Map)를 여러 논리적 구역이 공유하고, 별도의 의미 계층(Semantic Layer)을 이용하여 경계와 정책을 정의할 수 있다. 이러한 방식은 층 전체에서 위치추정 기하 구조가 연속적이지만 영역에 따라 내비게이션 동작이 달라지는 경우에 유용하다. 속도 제한, 접근 권한, 선호 경로, 교통 방향, 안전 여유 공간 등을 기본 점유 격자를 복제하지 않고 변경할 수 있다.

일부 시설에서는 동일한 층 내부에서도 지역 지도 전환(Local Map Switching)을 사용하는 것이 유리하다. 매우 큰 환경에서는 하나의 고해상도 지도를 저장하고 배포하며 처리하는 비용이 증가할 수 있다. 시설을 서로 중첩되는 여러 지역 지도로 분할하면 활성 메모리와 계산 부하를 제한할 수 있다. 로봇은 현재 위치한 영역에 따라 필요한 지도를 불러오거나 활성화하며, 전이 영역은 안정적인 위치추정 인계(Localization Handover)를 위한 충분한 기하학적 중첩을 제공한다.

지도 중첩(Map Overlap)은 의도적으로 설계해야 한다. 서로 인접한 두 지역 지도가 공통 기하 구조가 거의 없는 경계에서 만난다면 지도 전환 중 위치추정 불확실성이 증가할 수 있다. 안정적인 벽, 기둥 또는 기타 식별 가능한 구조가 포함된 중첩 영역(Overlap Region)을 확보하면 두 지도 문맥 모두에서 유효한 관측을 얻을 수 있다. 이를 통해 지도 사이의 변환 관계를 검증하고 전환 과정에서 발생하는 불연속을 감소시킬 수 있다.

좌표 변환(Coordinate Transformation)은 다중 지도 관리의 핵심이다. 각각의 지역 지도는 독립적인 원점을 정의할 수 있지만 상위 수준의 경로 계획에서는 공통 건물 또는 사이트 기준 좌표계(Global Reference Frame)가 필요할 수 있다. 알려진 변환 관계를 이용하여 층 및 구역 좌표를 이러한 전역 기준과 연결할 수 있다. 시스템은 물리적으로 의미 있는 변환과 독립적인 SLAM 세션에서 임의로 생성된 지도 원점을 명확하게 구분해야 한다.

독립적인 지도작성 세션(Independent Mapping Session)은 서로 다른 회전, 병진 또는 누적 오차를 가진 지도를 생성할 수 있다. 두 지도의 이미지가 시각적으로 정렬되어 보인다는 이유만으로 이들을 연결해서는 안 된다. 지도 사이의 관계는 측량된 랜드마크(Surveyed Landmark), 알려진 전이 자세, 신뢰성 높은 스캔 정합, 기준 마커(Fiducial Reference) 또는 기타 검증된 정합 방법(Registration Method)을 사용하여 설정해야 한다. 잘못된 지도 정합은 각 개별 지도가 내부적으로 정확하더라도 경로 계획 오류를 발생시킬 수 있다.

여러 지도를 연결하는 전역 경로 계획(Global Route Planning)은 자연스럽게 계층적 구조를 갖는다. 위상 경로 계획기(Topological Planner)는 예를 들어 구역 A, 엘리베이터 2, 3층, 구역 D와 같은 영역의 이동 순서를 결정할 수 있다. 각 영역 내부에서는 미터 단위 경로 계획기(Metric Planner)가 활성 점유 격자와 지역 비용지도(Local Costmap)를 이용하여 충돌 없는 경로를 생성한다. 위상 계획과 미터 단위 계획을 분리하면 본질적으로 이산적인 정보를 하나의 거대한 점유 격자에 억지로 포함시킬 필요가 없다.

전이 엣지(Transition Edge)는 기하학적 연결성뿐만 아니라 운용 비용(Operational Cost)을 포함할 수 있다. 엘리베이터에는 예상 대기시간이 존재할 수 있고, 출입문은 특정 시간에만 사용할 수 있으며, 좁은 통로는 대형 로봇이 통과하기 어려울 수 있다. 경로 계획은 이러한 비용과 제약조건을 활용하여 단순히 기하학적으로 가장 짧은 경로가 항상 최적의 운용 경로라고 가정하지 않고 여러 전이 후보 가운데 적절한 경로를 선택할 수 있다.

로봇의 기능 및 물리적 능력(Robot Capability)도 표현해야 한다. 소형 AMR은 좁은 출입문을 통과할 수 있지만 대형 적재 로봇은 통과하지 못할 수 있으며, 일부 로봇은 화물용 엘리베이터(Freight Elevator)는 사용할 수 있지만 승객용 엘리베이터는 사용할 수 없을 수 있다. 전이 및 구역 메타데이터에 로봇 크기, 적재 제한(Payload Restriction), 경사 한계(Slope Limit), 접근 등급(Access Class), 필요한 인터페이스를 포함하면 경로 생성 단계에서 플랫폼 호환성을 미리 판단할 수 있다.

위치추정 신뢰도(Localization Confidence)는 지도 전환에도 영향을 주어야 한다. 구역 경계 또는 층간 전이에 접근하는 로봇은 현재 자세 추정이 지도 인계를 실행하기에 충분히 신뢰할 수 있는지 판단해야 한다. 전환 이후에는 목적지 위치추정 시스템이 허용 가능한 신뢰 수준에 도달한 이후에 로봇이 계속 이동해야 한다. 신뢰도가 낮은 상태가 지속되면 복구 동작(Recovery Behavior)을 통해 재배치, 추가 스캔 또는 운영자 지원을 요청할 수 있다.

다중 지도 환경에서는 장애 복구(Failure Recovery)가 더욱 복잡해진다. 시스템은 로봇의 자세뿐만 아니라 현재 사용해야 하는 올바른 지도 식별자(Map Identity)까지 판단해야 하기 때문이다. 예를 들어 엘리베이터 로비 근처에서 로봇이 재시작되면 여러 층의 유사한 기하 구조를 관측할 수 있다. 엘리베이터 상태, 기압 센싱(Barometric Sensing), 인프라 통신(Infrastructure Communication), 시각 마커(Visual Marker), 임무 문맥(Mission Context) 등의 층 정보를 이용하면 기하학적 위치추정을 수행하기 전에 이러한 모호성을 줄일 수 있다.

반복되는 건축 구조(Repeated Architectural Pattern)는 다층 건물에서 특히 문제가 된다. 여러 층의 엘리베이터 로비와 복도가 거의 동일할 수 있기 때문에 스캔 정합은 잘못된 지도에서도 그럴듯한 자세를 생성할 수 있다. 따라서 지도 식별자는 단순히 지역 라이다 기하 구조만으로 추론하기보다 이산 상태(Discrete State)로 관리하는 것이 적절하다. 환경 센싱과 인프라 및 임무 정보를 결합하면 보다 안전한 위치추정 전략을 구성할 수 있다.

지도 버전 관리(Map Version Management)는 층과 구역을 서로 독립적으로 갱신할 수 있기 때문에 필수적이다. 한 층의 리모델링으로 다른 모든 지도를 다시 생성할 필요는 없지만 시설 수준 지도 데이터베이스(Facility-Level Map Database)는 어떤 버전이 전이 정의 및 의미 주석(Semantic Annotation)과 호환되는지를 기록해야 한다. 예를 들어 엘리베이터 출구의 기하 구조가 변경되면 목적 층 지도와 관련된 전이 자세를 함께 갱신해야 할 수 있다.

지도 서버(Map Server) 또는 지도 관리 서비스(Map-Management Service)는 사용 가능한 지도 목록을 유지하고 요청에 따라 적절한 자원을 제공할 수 있다. 지도 카탈로그(Map Catalog)에는 점유 격자, 메타데이터, 의미 구역, 전이 정의, 위치추정 파라미터, 버전 정보 등이 포함될 수 있다. 인접한 지도를 미리 적재(Preloading)하면 지도 전환 지연시간을 줄일 수 있으며 사용 빈도가 낮은 지도는 경로에서 필요할 때까지 적재하지 않을 수 있다.

플릿 시스템(Fleet System)은 여기에 추가적인 조정 계층(Coordination Layer)을 제공한다. 여러 AMR이 엘리베이터, 출입문, 좁은 복도 또는 이송 구역을 공유하면 이러한 전이 자원에 대한 예약(Reservation)과 스케줄링(Scheduling)이 필요할 수 있다. 지도는 전이가 어디에 존재하는지를 표현하고 플릿 관리는 로봇이 언제 해당 전이를 사용할 수 있는지를 결정한다. 이러한 책임을 분리하면 공간 표현과 교통 조정을 모든 기능에 강하게 결합하지 않고 독립적으로 발전시킬 수 있다.

검증(Validation)은 각각의 점유 격자를 독립적으로 시험하는 데 그치지 않고 여러 지도를 연결하는 전체 임무(Cross-Map Mission)를 포함해야 한다. 로봇에게 한 구역 또는 층에서 다른 구역이나 층으로 이동하도록 명령하고 접근, 전이, 지도 전환, 재위치추정, 최종 내비게이션까지 전체 과정을 평가해야 한다. 사용할 수 없는 엘리베이터, 차단된 전이, 위치추정 실패, 오래된 지도, 통신 손실, 중단된 전이에서의 복구도 시험해야 한다.

시뮬레이션(Simulation)과 기록된 임무 재생(Recorded Mission Replay)은 실제 배포 전에 계층적 경로 계획과 전이 논리를 검증하는 데 유용하다. 가상의 장애 상황을 통해 경로 계획기가 대체 엘리베이터나 다른 경로를 선택하는지 시험할 수 있으며, 기록된 센서 데이터를 이용하여 목적 지도에서의 재위치추정을 검증할 수 있다. 이러한 시험은 개별 지도 이미지만 검토해서는 발견하기 어려운 시스템 수준 장애(System-Level Failure)를 확인하는 데 도움이 된다.

따라서 자율이동로봇에서 다층 및 다중 구역 지도작성은 단순히 여러 개의 점유 격자를 저장하는 방법이 아니다. 이는 미터 단위 지도(Metric Map), 의미 구역, 위상 연결성(Topological Connectivity), 좌표 변환, 전이 절차, 위치추정 인계, 지도 버전, 로봇 기능을 결합하는 공간 관리 아키텍처(Spatial Management Architecture)이다. 각 계층은 대규모 내비게이션의 서로 다른 문제를 담당한다.

강건한 구현(Robust Implementation)은 평면 기하 구조가 적합한 영역에서는 지역 2D SLAM(Local 2D SLAM)을 유지하면서 하나의 점유 격자로 표현할 수 없는 관계를 상위 수준 구조로 추가한다. 층 지도는 미터 단위 위치추정을 제공하고, 구역은 운용 의미를 표현하며, 전이는 분리된 공간을 연결하고, 계층적 경로 계획(Hierarchical Planning)은 이들 사이의 이동을 조정한다. 이러한 메커니즘을 결합하면 전체 환경을 하나의 인위적인 평면 지도에 강제로 통합하지 않고도 AMR이 복잡한 시설을 안정적으로 탐색하고 이동할 수 있다.

## 03.09. 2D SLAM Benchmark EVO ATE RTE Evaluation [w/Code]

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

2D SLAM 벤치마킹(Benchmarking)은 점유 지도(Occupancy Map)가 깔끔하게 보이는지를 시각적으로 확인하는 것만으로는 충분하지 않다. SLAM 시스템은 시간에 따라 변화하는 로봇 궤적(Robot Trajectory)을 추정하므로 그 정확도를 신뢰할 수 있는 기준 궤적(Reference Trajectory)과 정량적으로 비교해야 한다. EVO와 같은 도구는 서로 다른 알고리즘과 실험 설정에 대해 궤적 동기화(Trajectory Synchronization), 정렬(Alignment), 오차 계산(Error Computation), 통계 분석(Statistical Analysis), 시각화(Visualization)를 수행하기 위한 표준화된 작업 흐름을 제공한다.

벤치마크(Benchmark)는 일반적으로 추정 궤적(Estimated Trajectory)과 타임스탬프가 포함된 자세로 표현된 실측 또는 기준 궤적(Ground-Truth or Reference Trajectory)으로 구성된다. 평면 SLAM(Planar SLAM)의 주요 상태는 \\(x\\), \\(y\\), 요각(Yaw)으로 구성되지만 궤적 파일에는 전체 SE(3) 자세가 저장될 수도 있다. 실측 정보는 모션 캡처 시스템(Motion-Capture System), 측량 랜드마크(Surveyed Landmark), 고정밀 GNSS/INS, 기준 SLAM 시스템 또는 평가 대상보다 높은 정확도를 가진 측정 인프라에서 획득할 수 있다.

오차 지표(Error Metric)를 계산하기 전에 두 궤적은 동일한 물리적 움직임을 나타내고 호환 가능한 좌표 규약(Coordinate Convention)을 사용해야 한다. 원점, 방향, 타임스탬프 정의, 축 방향 또는 단위의 차이는 실제 SLAM 성능과 관계없는 큰 겉보기 오차를 발생시킬 수 있다. 따라서 벤치마크 준비 과정에서는 좌표계(Coordinate Frame), 타임스탬프의 단조 증가성(Timestamp Monotonicity), 궤적 지속시간, 샘플링 주파수(Sampling Frequency), 두 데이터셋이 동일한 이동 구간을 포함하는지 확인해야 한다.

타임스탬프 연관(Timestamp Association)은 추정 자세와 기준 자세가 동일한 주파수로 기록되는 경우가 드물기 때문에 특히 중요하다. 평가 소프트웨어는 일반적으로 최대 허용 시간 차이를 설정하여 타임스탬프에 따라 자세를 연결하고 필요한 경우 보간(Interpolation)을 사용한다. 시간 오프셋(Temporal Offset)이 잘못되면 공간적으로 정확한 궤적도 부정확하게 보일 수 있으며, 특히 로봇이 빠르게 이동하거나 급격하게 회전할 때 이러한 현상이 두드러진다. 따라서 기하학적 오차를 해석하기 전에 시계 동기화(Clock Synchronization)를 검증해야 한다.

궤적 정렬(Trajectory Alignment)은 평가 대상인 위치추정 오차와 관계없는 좌표계 차이를 제거한다. 실험 목적에 따라 병진(Translation)과 회전(Rotation)을 이용하여 궤적을 정렬하거나 스케일(Scale)까지 추정하는 보다 일반적인 유사 변환(Similarity Transformation)을 사용할 수 있다. 미터 단위 2D 라이다 SLAM(Metric 2D LiDAR SLAM)은 물리적으로 의미 있는 미터 단위의 움직임을 추정해야 하므로 임의적인 스케일 보정은 일반적으로 사용하지 않는 것이 적절하다.

절대 궤적 오차(Absolute Trajectory Error, ATE)는 EVO에서 일반적으로 절대 자세 오차(Absolute Pose Error)를 통해 평가되며, 정렬된 추정 궤적과 기준 궤적 사이의 동일한 타임스탬프에서의 차이를 측정한다. \\(T_i\^{ref}\\)와 \\(T_i\^{est}\\)가 각각 기준 자세와 추정 자세를 나타낸다면 선택된 정렬 규칙을 적용한 이후 오차 변환(Error Transform)은 개념적으로 \\(E_i=(T_i\^{ref})\^{-1}T_i\^{est}\\)와 같이 표현할 수 있다.

절대 오차의 병진 성분(Translational Component)은 전체 궤적에서 추정된 로봇 위치가 기준 위치로부터 얼마나 벗어나는지를 나타낸다. 일반적인 요약 통계에는 평균제곱근오차(Root Mean Square Error, RMSE), 평균(Mean), 중앙값(Median), 표준편차(Standard Deviation), 최솟값, 최댓값, 특정 백분위수(Percentile)가 포함된다. RMSE는 큰 오차에 더 높은 가중치가 적용되기 때문에 간결한 벤치마크 값으로 유용하지만 추가 통계와 궤적 그래프 없이 단독으로 보고해서는 안 된다.

절대 회전 오차(Absolute Rotational Error)는 내비게이션 문제가 주로 평면 운동을 대상으로 하는 경우에도 중요할 수 있다. 작은 요각 오차(Yaw Error)는 로봇이 전진함에 따라 횡방향 위치 오차를 증가시키고 지도작성 과정에서 벽이 눈에 띄게 왜곡되는 원인이 될 수 있다. 따라서 가능하면 병진과 회전을 별도로 평가해야 하며, 특히 서로 다른 스캔 정합, 오도메트리 또는 관성 제약조건을 사용하는 알고리즘을 비교할 때 중요하다.

절대 오차는 전역 궤적 일관성(Global Trajectory Consistency)을 측정하는 데 유용하지만 오차가 지역적으로 어떻게 누적되는지는 설명하지 못한다. 전체적인 전역 오차가 중간 수준이더라도 특정 구간에서 단기 운동 추정이 불안정할 수 있다. 반대로 지역 추적(Local Tracking)은 정확하지만 장기적인 드리프트가 지속적으로 증가할 수도 있다. 상대 궤적 오차(Relative Trajectory Error, RTE) 또는 상대 자세 오차(Relative Pose Error)는 선택된 시간 또는 공간 구간에서 운동 증분(Motion Increment)을 비교함으로써 이러한 한계를 보완한다.

두 궤적 인덱스 \\(i\\)와 \\(j\\)에 대해 기준 궤적의 상대 운동(Relative Motion)을 추정 궤적의 상대 운동과 비교할 수 있다. 개념적으로 이 지표는 \\((T_i\^{ref})\^{-1}T_j\^{ref}\\)와 \\((T_i\^{est})\^{-1}T_j\^{est}\\) 사이의 차이를 평가한다. 따라서 하나의 전역 좌표계에 대한 의존성보다 지역 드리프트(Local Drift)를 강조하며 오도메트리와 유사한 운동 추정 성능과 스캔 정합 안정성(Scan-Matching Stability)을 평가하는 데 특히 유용하다.

상대 오차는 선택한 평가 구간(Interval)에 크게 영향을 받는다. 연속된 자세 사이의 오차를 평가하면 매우 짧은 시간의 일관성을 측정할 수 있지만 측정 잡음의 영향을 크게 받을 수 있다. 더 긴 구간을 사용하면 내비게이션 관점에서 의미 있는 거리 또는 시간 동안 누적되는 드리프트를 확인할 수 있다. 따라서 엄밀한 벤치마크에서는 상대 오차가 프레임당, 초당, 미터당 또는 다른 고정 구간을 기준으로 계산되었는지 명확하게 기록해야 한다.

AMR에서는 거리 정규화 드리프트(Distance-Normalized Drift)가 직관적인 공학적 평가 지표가 될 수 있다. 병진 오차를 주행 거리 대비 비율로 표현할 수 있으며 회전 드리프트(Rotational Drift)는 미터당 또는 일정한 경로 길이당 값으로 표현할 수 있다. 이러한 지표는 길이가 서로 다른 궤적을 비교하기 쉽게 한다. 그러나 실험 사이에서 정규화 정의를 동일하게 유지해야 보고된 값이 동일한 물리량을 의미한다.

EVO는 일반적인 궤적 형식(Trajectory Format)을 지원하며 추정 궤적과 기준 궤적을 검사하고 정렬하며 비교하고 그래프로 표현하기 위한 도구를 제공한다. 일반적인 작업 흐름에서는 SLAM 자세를 지원되는 형식으로 변환하거나 내보내고 타임스탬프와 좌표 규약을 검증한 후 궤적 연관(Trajectory Association)을 수행한다. 이후 의도한 정렬 방법을 적용하고 모든 실험에 동일한 옵션을 사용하여 절대 및 상대 자세 오차를 계산한다.

정렬 설정(Alignment Setting)은 벤치마크 결과를 크게 변화시킬 수 있으므로 반드시 기록해야 한다. 전체 기준 자세를 이용하여 전체 궤적을 정렬하면 일정한 초기 좌표 차이를 제거할 수 있으며 이는 궤적의 형상 자체를 평가하는 경우에 적절하다. 초기 일부 구간만 정렬하는 것은 이후 발생하는 드리프트가 보정되지 않으므로 다른 평가 의미를 갖는다. 유사 변환 정렬(Similarity Alignment)은 스케일 오차까지 숨길 수 있으므로 평가 대상 방법에 본질적인 스케일 모호성(Scale Ambiguity)이 존재하는 경우에만 사용해야 한다.

수치 지표가 제공되더라도 궤적 그래프(Trajectory Plot)는 여전히 중요하다. 추정 경로와 기준 경로를 중첩하면 어느 위치에서 궤적이 벌어지기 시작하는지, 급격한 회전 근처에서 오차가 발생하는지, 루프 폐쇄(Loop Closure)가 누적 드리프트를 보정하는지 또는 잘못된 루프 폐쇄가 갑작스러운 전역 변형(Global Deformation)을 발생시키는지를 확인할 수 있다. 시간 및 거리 대비 오차 그래프는 하나의 RMSE 값만으로는 확인하기 어려운 실패 형태(Failure Mode)를 보여줄 수 있다.

SLAM 벤치마킹에서는 루프 폐쇄를 명시적으로 분석해야 한다. 시스템은 탐색 과정에서 상당한 드리프트를 누적한 후 이전에 방문한 영역을 인식하면서 궤적을 보정할 수 있다. 따라서 최종 절대 오차는 작더라도 루프 폐쇄 이전의 온라인 위치추정(Online Localization) 성능은 좋지 않았을 수 있다. 실시간 AMR 운용을 평가할 때는 가능하다면 온라인 궤적 품질과 사후 최적화된 궤적(Retrospectively Optimized Trajectory)의 품질을 구분하여 평가해야 한다.

지도 품질(Map Quality)과 궤적 정확도(Trajectory Accuracy)는 서로 관련되어 있지만 동일한 개념은 아니다. 체계적인 오차를 포함한 궤적에서도 시각적으로 선명한 점유 지도를 생성할 수 있으며, 반대로 정량적으로 정확한 궤적을 얻더라도 센서 보정이나 점유 파라미터가 부적절하면 잡음이 많은 지도가 생성될 수 있다. 따라서 종합적인 2D SLAM 벤치마크는 궤적 지표와 함께 지도 일관성(Map Consistency) 및 내비게이션에 필요한 기하학적 품질도 평가해야 한다.

실측 데이터의 불확실성(Ground-Truth Uncertainty)은 의미 있는 평가 정확도의 하한을 결정한다. 기준 시스템 자체가 센티미터 수준의 불확실성을 가진다면 그보다 작은 차이를 명확한 알고리즘 성능 향상으로 해석해서는 안 된다. 따라서 기준 궤적에 대해서도 자체 정확도, 갱신 주기(Update Rate), 측정 범위(Coverage), 실패 조건을 파악해야 한다. 벤치마크의 정밀도는 기준으로 사용하는 측정 시스템의 신뢰성을 초과할 수 없다.

반복성(Repeatability)도 중요한 요구조건이다. 한 번의 성공적인 주행만으로 강건한 SLAM 성능을 입증할 수는 없다. 동적 객체, 초기화 상태, CPU 스케줄링, 센서 잡음, 알고리즘의 무작위성에 따라 결과가 달라질 수 있기 때문이다. 동일하거나 비교 가능한 경로에서 여러 번의 주행을 평가하고 실험 전체의 중심 경향(Central Tendency)과 변동성(Variability)을 함께 보고해야 한다.

벤치마크 경로(Benchmark Route)는 단순한 개방형 루프만 사용하는 대신 실제 운용 환경에서 발생하는 대표적인 난제를 포함해야 한다. 긴 복도, 교차로, 개방 공간, 반복 구조, 좁은 통로, 급격한 회전, 동적 교통 환경, 루프 폐쇄 구간 등이 유용한 시험 조건이다. 데이터셋에는 누적 드리프트를 확인할 수 있을 만큼 충분한 궤적 길이와 전역 최적화가 일관성을 성공적으로 회복하는지를 평가할 수 있는 충분한 재방문 구간이 포함되어야 한다.

알고리즘 비교(Algorithm Comparison)를 수행할 때는 가능한 한 동일한 입력 조건을 사용해야 한다. GMapping, Hector SLAM, Cartographer, SLAM Toolbox 또는 다른 시스템은 동일한 센서 데이터셋, 센서 보정, 시간 구간, 기준 궤적을 사용하여 처리해야 한다. 전처리, 라이다 거리 제한 또는 제외된 궤적 구간이 서로 다르다면 이를 기록해야 한다. 그렇지 않으면 벤치마크가 알고리즘 성능이 아니라 실험 구성 차이를 측정하게 될 수 있다.

기하학적 정확도와 함께 계산 성능(Computational Performance)도 평가해야 한다. CPU 사용률, 메모리 소비량, 처리 지연시간(Processing Latency), 갱신 주파수, 실시간 계수(Real-Time Factor)는 해당 알고리즘이 임베디드 AMR 컴퓨터에서 실제로 사용할 수 있는지를 결정한다. ATE가 조금 더 우수하더라도 지연시간이 지나치게 큰 알고리즘은 정확도가 약간 낮더라도 결정론적인 실시간 동작(Deterministic Real-Time Operation)을 유지하는 시스템보다 실용성이 떨어질 수 있다.

많은 파라미터 설정이나 알고리즘을 비교할 때 벤치마크 자동화(Benchmark Automation)는 일관성을 향상시킨다. 스크립트를 이용하면 모든 실험에 동일한 궤적 변환, 타임스탬프 연관, 정렬, EVO 명령, 통계 추출, 그래프 생성을 적용할 수 있다. 결과를 설정 파일(Configuration File) 및 소프트웨어 버전과 함께 저장하면 관측된 성능 향상이 어떤 파라미터 또는 구현 변경에서 발생했는지를 추적할 수 있다.

유용한 평가 기록(Evaluation Record)에는 데이터셋 식별자, SLAM 알고리즘과 버전, 파라미터 설정, 센서 보정, 궤적 형식, 정렬 방법, 연관 허용오차(Association Tolerance), 지표 정의, EVO 버전이 포함되어야 한다. 이러한 메타데이터가 없으면 수치 결과를 재현하기 어려우며 원본 궤적 파일을 보존하고 있더라도 수개월 후의 비교에서 실험 조건이 모호해질 수 있다.

생산용 AMR에서는 학술적인 벤치마크 관행만을 기준으로 수용 임계값(Acceptance Threshold)을 설정하기보다 실제 내비게이션 요구조건으로부터 이를 도출해야 한다. 위치추정 정확도는 통로 여유 공간(Corridor Clearance), 도킹(Docking), 장애물 회피, 경로 실행에 충분해야 한다. 따라서 짧은 시간 동안 발생하는 큰 위치추정 오차가 지속적인 작은 드리프트보다 더 큰 운용 위험을 발생시킬 수 있으므로 평균 RMSE뿐만 아니라 최대 오차와 높은 백분위수 영역의 성능도 중요하다.

체계적인 2D SLAM 벤치마크는 전역 정확도를 위한 ATE, 지역 드리프트를 위한 RTE 또는 상대 자세 오차, 실패 원인 해석을 위한 궤적 및 오차 그래프, 강건성 평가를 위한 반복 시험, 실제 배포 가능성을 위한 계산 성능 지표를 결합한다. EVO는 이러한 궤적 분석의 상당 부분을 수행할 수 있는 실용적인 프레임워크를 제공하지만 의미 있는 결론을 얻기 위해서는 정확한 동기화, 정렬, 기준 데이터 품질, 실험 설계가 반드시 뒷받침되어야 한다.

최종적인 목적은 단순히 가장 작은 하나의 오차 수치를 얻는 것이 아니라 SLAM 설정이 목표 로봇에 대해 정확하고 안정적이며 재현 가능하고 계산적으로 지속 가능한 위치추정(Localization)을 제공하는지를 판단하는 것이다. 정량적인 궤적 평가(Quantitative Trajectory Evaluation)는 SLAM 튜닝을 시각적인 판단에서 측정 가능한 공학적 평가로 전환하며, 알고리즘과 파라미터 설정을 주관적인 지도 외형이 아니라 반복 가능한 근거를 이용하여 비교할 수 있도록 한다.

## 03.10. Indoor AMR Production 2D SLAM Deployment Case

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

생산 환경의 실내 자율이동로봇(Autonomous Mobile Robot, AMR)에 2D SLAM을 적용하려면 실험실에서 지도작성(Mapping)을 시연하는 것과는 다른 공학적 관점이 필요하다. 목표는 단순히 정확한 점유 격자(Occupancy Grid)를 생성하는 것이 아니라 변화하는 교통 상황, 적재 상태, 조명, 바닥 조건, 네트워크 가용성에서도 일상적인 임무 전체에 걸쳐 신뢰성 높은 위치추정(Localization)을 유지하는 것이다. 따라서 생산 적용에서는 SLAM을 통합된 내비게이션 및 운용 시스템의 일부로 다루어야 한다.

일반적인 실내 AMR은 2D 라이다(2D LiDAR), 휠 인코더(Wheel Encoder), 모터 피드백(Motor Feedback), 관성 측정 장치(Inertial Measurement Unit, IMU)를 결합한다. 라이다는 벽, 기둥, 랙, 장비와 같은 안정적인 기하 구조를 관측하고 휠 오도메트리(Wheel Odometry)는 높은 주파수의 단기 운동 추정값을 제공한다. IMU는 가속이나 회전 중 회전 추정(Rotational Estimation)을 개선할 수 있다. SLAM 튜닝을 시작하기 전에 이러한 측정값은 정확한 타임스탬프와 보정된 좌표 변환(Coordinate Transformation)을 공유해야 한다.

센서 배치(Sensor Placement)는 위치추정 품질에 큰 영향을 준다. 라이다가 지나치게 낮게 설치되면 안정적인 시설 구조 대신 바퀴, 팔레트 또는 일시적인 바닥 수준 장애물을 관측할 수 있으며, 적재물이나 로봇 구조물이 시야(Field of View)를 가리는 위치에 설치되면 유용한 특징을 잃을 수 있다. 생산용 설계에서는 충분한 수평 시야, 기계적 강성(Mechanical Rigidity), 충격 보호, 유지보수 또는 센서 교체 이후에도 반복 가능한 장착 기하 구조를 확보해야 한다.

초기 지도작성 임무(Initial Mapping Mission)는 가능하면 통제된 조건에서 수행해야 한다. 주요 출입문은 실제 운용을 대표하는 상태로 유지하고 일시적인 장애물은 최소화하며 로봇이 내비게이션에 필요한 모든 복도와 교차로를 주행하도록 해야 한다. 지도작성 경로에는 재방문 구간을 포함하여 루프 폐쇄(Loop Closure)가 누적 드리프트를 보정할 수 있도록 하고 시설의 서로 멀리 떨어진 영역들이 기하학적으로 일관되는지를 확인할 수 있어야 한다.

생산용 지도(Production Map)는 SLAM 주행이 끝났다고 즉시 배포해서는 안 된다. 생성된 궤적과 점유 격자에서 중복된 벽, 왜곡된 복도, 잘못된 루프 폐쇄(False Loop Closure), 원인을 설명할 수 없는 불연속, 일시적인 장애물을 먼저 검사해야 한다. 지도 후처리(Map Post-Processing)를 통해 제한적인 인공물(Artifact)을 제거할 수 있지만 체계적인 변형이 존재한다면 수동으로 외형을 수정하는 대신 보정, 동기화 또는 SLAM 설정을 수정한 후 다시 지도작성해야 한다.

검증 이후 기하학적 지도(Geometric Map)는 통제되는 위치추정 기준(Localization Reference)이 된다. 진입 금지 구역(No-Go Zone), 속도 제한 구역(Speed-Limited Region), 충전 스테이션(Charging Station), 도킹 자세(Docking Pose), 선호 경로(Preferred Route), 제한 작업 셀(Restricted Work Cell)과 같은 운용 정보는 일반적으로 별도의 내비게이션 또는 의미 계층(Semantic Layer)에서 관리해야 한다. 이를 통해 스캔 기반 위치추정에 사용되는 물리적 기하 구조를 변경하지 않고도 교통 규칙과 생산 정책을 수정할 수 있다.

일상적인 생산 운용에서는 일반적으로 검증된 지도를 지속적으로 다시 작성하기보다 해당 지도를 기준으로 위치추정을 수행한다. 입력되는 라이다 스캔은 저장된 기하 구조와 정합되고 오도메트리는 관측 사이의 단기 운동을 예측한다. 이러한 아키텍처는 산업 시설에서 빈번하게 등장하는 일시적인 팔레트, 작업자, 카트, 지게차 및 기타 동적 객체(Dynamic Object)가 기준 지도를 오염시키는 것을 방지한다.

위치추정 초기화(Localization Initialization)는 중요한 운용 단계이다. 로봇을 시작할 때 알려진 충전 위치, 도킹 스테이션, 수동으로 지정한 초기 자세(Initial Pose), 또는 전역 재위치추정(Global Relocalization) 절차를 사용할 수 있다. 자율 이동을 시작하기 전에 관측된 라이다 기하 구조가 초기화된 자세와 충분히 일치하는지 검증해야 한다. 수치적으로 유효한 자세 추정값이 존재한다는 이유만으로 위치추정 신뢰도(Localization Confidence)를 확인하지 않고 해당 위치를 신뢰해서는 안 된다.

생산 환경에는 반복적인 기하 구조(Repetitive Geometry)가 자주 존재한다. 긴 창고 통로, 반복되는 랙, 동일한 형태의 기둥, 유사한 생산 셀은 여러 개의 그럴듯한 스캔 정합 위치를 만들 수 있다. 오도메트리 이력(Odometry History), 알려진 시작 영역, 임무 문맥(Mission Context), 제한된 초기화 영역, 기준 마커(Fiducial Infrastructure), 추가 센싱을 활용하면 이러한 모호성을 줄일 수 있다. 명목상의 자세 오차를 최소화하는 것보다 높은 신뢰도를 가진 잘못된 위치추정을 방지하는 것이 더 중요할 수 있다.

동적 장애물(Dynamic Obstacle)은 정적 SLAM 지도에 포함시키기보다 주로 지역 인지(Local Perception)와 비용지도(Costmap)를 통해 처리해야 한다. 작업자나 지게차가 지도상의 통로를 일시적으로 차단하더라도 구조 지도 자체는 여전히 정확할 수 있다. 내비게이션 시스템은 지속적인 위치추정 지도와 실시간 장애물 관측을 결합하여 단기 객체가 장기 공간 지식(Long-Term Spatial Knowledge)을 오염시키지 않으면서 현재 상황에 맞게 경로 계획을 수행할 수 있다.

휠 슬립(Wheel Slip)은 생산 환경에서 위치추정 성능 저하를 발생시키는 일반적인 원인이다. 먼지, 젖은 바닥, 신축 이음부(Expansion Joint), 경사로, 불균일한 표면, 급가속, 적재 변화 등은 바퀴 회전과 실제 로봇 운동 사이의 관계를 변화시킬 수 있다. 라이다 스캔 정합은 중간 수준의 오도메트리 드리프트를 보정할 수 있지만 심한 슬립은 스캔 정합기의 탐색 범위를 증가시킨다. 따라서 운동 제어(Motion Control)와 위치추정은 오도메트리를 완벽한 독립 입력으로 간주하지 않고 함께 설계해야 한다.

적재량 변화(Payload Variation)도 간접적으로 SLAM에 영향을 줄 수 있다. 무거운 화물을 적재한 AMR은 무부하 상태와 비교하여 타이어 변형, 서스펜션 압축, 가속 응답 또는 휠 슬립 특성이 달라질 수 있다. 센서 마스트(Sensor Mast) 또는 섀시가 변형된다면 실질적인 라이다 좌표 변환도 달라질 수 있다. 따라서 생산 환경 검증은 무부하 개발 차량만을 대상으로 하지 않고 실제 운용을 대표하는 적재 조건을 포함해야 한다.

위치추정 성능은 전체 운용 경로(Operational Route)를 기준으로 측정해야 한다. 실측 정보(Ground Truth)를 사용할 수 있다면 절대 및 상대 궤적 오차(Absolute and Relative Trajectory Error), 스캔 정합 품질, 위치추정 공분산 또는 신뢰도, 복구 발생 빈도, CPU 사용률, 처리 지연시간(Processing Latency) 등을 평가할 수 있다. 도킹 반복정밀도(Docking Repeatability)와 경로 추종(Path Tracking) 성능도 응용 수준의 중요한 지표가 된다. 위치추정은 로봇의 실제 임무 수행에 필요한 정확도를 제공할 때 의미가 있기 때문이다.

컴퓨팅 플랫폼(Compute Platform)은 충분한 실시간 처리 여유(Real-Time Margin)를 유지해야 한다. SLAM 또는 위치추정은 장애물 탐지, 전역 및 지역 경로 계획, 플릿 통신(Fleet Communication), 안전 모니터링, 로깅(Logging), 응용 소프트웨어와 프로세서 자원을 공유한다. 평균 CPU 사용률만으로는 충분하지 않으며 루프 폐쇄, 지도 로딩, 밀집된 라이다 환경, 통신량 급증 등 계산 부하가 최대가 되는 조건에서 최악 지연시간(Worst-Case Latency)과 스케줄링 동작을 확인해야 한다.

위치추정 성능 저하 상태(Degraded-Localization State)는 명시적으로 탐지할 수 있어야 한다. 지속적인 스캔 정합 실패, 빠르게 증가하는 불확실성, 오도메트리와 지도 기반 자세 사이의 불일치, 비현실적인 자세 점프(Pose Jump), 반복적인 관측 정렬 실패 등이 지표가 될 수 있다. 신뢰도가 운용 임계값 이하로 떨어지면 정상적인 내비게이션을 계속하는 것보다 정지하는 것이 안전할 수 있다. 로봇은 실패한 추정값을 계속 신뢰하지 않고 정의된 복구 상태(Recovery State)로 전환해야 한다.

복구 동작(Recovery Behavior)은 문제의 심각도에 따라 단계적으로 구성할 수 있다. 먼저 로봇의 속도를 줄이고 추가적인 스캔 정합을 시도한 후 필요하면 정지하여 보다 넓은 범위의 재위치추정을 수행할 수 있다. 신뢰도가 충분히 회복되지 않으면 안전한 조건에서 알려진 위치추정 영역으로 복귀하거나 운영자 지원(Operator Assistance)을 요청할 수 있다. 위치 식별에 대한 신뢰성이 확보되지 않은 동안에는 일시적인 장애가 영구 지도 데이터를 손상시키지 않도록 장기 지도 갱신을 중지해야 한다.

도킹(Docking)은 일반적인 복도 주행보다 높은 최종 정확도가 요구될 수 있으므로 별도로 처리하는 것이 적절하다. 전역 2D 위치추정은 AMR을 충전기, 컨베이어, 작업 스테이션 또는 자재 이송 인터페이스 근처까지 유도하고, 최종 접근에서는 반사 마커(Reflective Marker), 시각 기준 마커(Visual Fiducial), 지역 라이다 특징(Local LiDAR Feature), 기계적 가이드(Mechanical Guide) 또는 다른 고정밀 상대 센싱 방법을 사용할 수 있다. 이러한 계층적 전략을 통해 모든 영역에서 불필요하게 높은 전역 지도 정밀도를 요구하지 않아도 된다.

지도 변경(Map Change)은 통제된 유지관리 절차(Controlled Maintenance Process)를 따라야 한다. 팔레트의 이동은 재지도작성(Remapping)의 근거가 되지 않지만 벽, 생산 장비 또는 영구 랙이 이동하면 저장된 지도와 현재 관측 사이에 지속적인 불일치가 발생할 수 있다. 구조적 변화에 대한 반복적인 증거가 확인되면 지도 검토를 수행하고 변경 규모와 안전 중요도에 따라 지역 지도 수정(Local Map Correction) 또는 새로운 지도작성 세션을 진행해야 한다.

모든 생산용 지도에는 관리되는 버전(Managed Version)과 호환성 기록(Compatibility Record)이 있어야 한다. 지도 버전은 해상도, 원점, SLAM 설정, 위치추정 파라미터, 의미 계층, 도킹 자세, 검증 결과와 연계하여 관리해야 한다. 새로운 지도를 배포한 이후에도 이전에 검증된 버전으로 롤백(Rollback)할 수 있어야 한다. 따라서 지도 배포는 소프트웨어 릴리스 관리(Software Release Management)와 유사한 구성 관리(Configuration Management) 규율을 적용해야 한다.

플릿 운용(Fleet Operation)에서는 지도 일관성이 더욱 중요하다. 동일한 시설에서 운용되는 여러 AMR은 서로 호환되는 기하학적 지도와 운용 주석(Operational Annotation)을 사용해야 한다. 서로 다른 원점, 오래된 진입 금지 구역 또는 일치하지 않는 도킹 좌표를 사용하는 로봇들이 존재하면 각 로봇이 자신의 지도에서 정확하게 위치추정하더라도 플릿 수준의 교통 조정이 실패할 수 있다. 따라서 지도와 주석의 배포는 중앙에서 통제되어야 한다.

생산 플릿은 지도 품질에 관한 유용한 증거도 제공할 수 있다. 여러 로봇에서 반복적으로 수집되는 위치추정 잔차(Localization Residual)를 분석하면 기하 구조가 변경된 영역, 라이다 관측이 지속적으로 모호한 영역 또는 휠 슬립이 빈번하게 발생하는 위치를 발견할 수 있다. 이러한 데이터는 권위 있는 지도(Authoritative Map)를 자동으로 수정하는 데 사용하기보다 유지관리 정보로 분석해야 한다. 사람 또는 통제된 자동 검증 절차를 통해 실제 지도 갱신이 필요한지를 판단할 수 있다.

네트워크 손실(Network Loss)이 발생하더라도 위치추정 기능이 즉시 사라져서는 안 된다. 플릿 서버와의 통신이 중단되더라도 핵심적인 지도 기반 위치추정과 안전에 필요한 지역 내비게이션은 가능한 한 로봇 내부에서 유지되어야 한다. 통신 장애 동안 플릿 명령, 교통 예약 또는 원격 모니터링 기능은 제한될 수 있지만 AMR은 안전하게 정지하거나 정의된 복구 정책을 수행할 수 있는 충분한 온보드 공간 인식 기능(Onboard Spatial Capability)을 유지해야 한다.

시험은 정상적인 운용 조건뿐만 아니라 어려운 생산 조건을 재현해야 한다. 검증 경로에는 좁은 통로, 개방 공간, 반복 구조, 교차로, 동적 교통, 급격한 회전, 일시적인 센서 가림, 다양한 적재 상태, 장시간 임무, 반복적인 루프 형태의 경로를 포함해야 한다. 재시작, 센서 중단, 위치추정 상실, 지도 불일치, 컴퓨팅 과부하, 네트워크 장애도 명시적인 시험 시나리오(Test Scenario)로 포함해야 한다.

기록된 센서 데이터(Recorded Sensor Data)는 소프트웨어 또는 파라미터 변경 이후의 회귀 시험(Regression Testing)에 유용하다. 검증된 대표 라이다, 오도메트리, IMU 및 기준 데이터를 각각의 후보 설정에 반복 재생할 수 있다. 이후 궤적 지표, 위치추정 실패, CPU 부하, 지도 정렬 성능을 자동으로 비교할 수 있다. 이를 통해 사소해 보이는 파라미터 변경이 기존에 안정적으로 동작하던 조건에서 예상하지 못한 성능 저하를 발생시키는 것을 방지할 수 있다.

생산 승인 기준(Production Acceptance Criteria)은 임무 요구조건(Mission Requirement)으로부터 도출해야 한다. 넓은 통로에서 자재를 운반하는 로봇은 자동화 장비에 정밀하게 도킹해야 하는 AMR보다 더 큰 위치추정 오차를 허용할 수 있다. 따라서 하나의 보편적인 SLAM 정확도 목표를 정의하기보다 위치추정 성능을 최소 여유 공간(Minimum Clearance), 정지 동작, 도킹 허용오차(Docking Tolerance), 경로 가용성, 운용 속도, 안전 아키텍처(Safety Architecture)와 연결하여 요구조건을 설정해야 한다.

따라서 생산 적용 주기(Production Deployment Cycle)는 일회성 지도작성 작업이 아니라 지속적인 공학 활동(Continuous Engineering)이 된다. 지도작성은 기하학적 기준을 확립하고, 검증은 해당 지도의 적합성을 확인하며, 위치추정은 실제 운용 중 지도를 사용한다. 플릿 데이터는 반복적으로 발생하는 취약점을 발견하고, 통제된 유지관리는 환경 변화에 대응하며, 회귀 시험은 업데이트 과정에서 기존 성능이 유지되도록 보호한다. 각 단계는 장기적인 내비게이션 신뢰성에 기여한다.

성공적인 실내 AMR 2D SLAM 적용은 최종적으로 지도의 외형이 아니라 운용 안정성(Operational Stability)으로 평가해야 한다. 로봇은 자신의 위치를 신뢰성 있게 파악하고, 그 정보가 불확실해지는 시점을 인식하며, 영구적인 공간 정보를 손상시키지 않고 복구할 수 있어야 한다. 또한 시설 환경이 변화하더라도 내비게이션 요구조건을 지속적으로 만족해야 한다. 따라서 생산 수준 SLAM(Production-Grade SLAM)은 기하학적 추정, 센서 공학, 검증, 구성 관리, 복구 로직, 운용 거버넌스(Operational Governance)를 하나의 유지관리 가능한 위치추정 시스템으로 통합하는 기술이다.
