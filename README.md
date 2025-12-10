# **Implementation of Kalman Filter**

### **NAME:** Mohamed Riyaz Ahamed

### **REGISTER NO.:** 212224240092

### **EX. NO.:** 5

<br>

# **Implementation of Kalman Filter**

## **Aim**

To construct a Python program that implements a **Kalman Filter** to predict the **position** and **velocity** of a moving object.

---

## **Algorithm**

1. **Define System Models**

   * State transition model **F**
   * Observation model **H**
   * Process noise covariance **Q**
   * Measurement noise covariance **R**
   * Initial state estimate **x₀**
   * Initial error covariance **P₀**

2. **Create a KalmanFilter object** using the above parameters.

3. **Simulate object motion** over several time steps and generate:

   * True states
   * Noisy measurements

4. **Prediction Step:**
   Use `kf.predict()` to estimate the next state.

5. **Update Step:**
   Incorporate the measurement using `kf.update(z)`.

6. **Store all estimated states** for comparison.

7. **Plot results** comparing **true position** and **estimated position**.

---

## **Program**

```python
import numpy as np
import matplotlib.pyplot as plt

class KalmanFi1ter:
    def __init__(self, F, H, Q, R, x0, P0):
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        s = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(s))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(self.F.shape[0]) - np.dot(K, self.H), self.P)

# Time step
dt = 0.1

# System matrices
F = np.array([[1, dt],
              [0, 1]])

H = np.array([[1, 0]])

Q = np.diag([0.1, 0.1])
R = np.array([[1]])

# Initial conditions
x0 = np.array([0, 0])
P0 = np.diag([1, 1])

# Create Kalman Filter object
kf = KalmanFi1ter(F, H, Q, R, x0, P0)

# Simulation
truestates = []
measurements = []

for i in range(100):
    truestates.append([i * dt, 1])  # true position increases linearly
    measurements.append(i * dt + np.random.normal(scale=1))  # noisy measurement

# Estimation
est_states = []

for z in measurements:
    kf.predict()
    kf.update(np.array([z]))
    est_states.append(kf.x)

# Plotting
plt.plot([s[0] for s in truestates], label="True")
plt.plot([s[0] for s in est_states], label="Estimated")
plt.legend()
plt.show()
```

---



<H3>Output:</H3>
<img width="746" height="532" alt="image" src="https://github.com/user-attachments/assets/0b23ea4a-1282-490b-b14f-682ddf5071a2" />

<H3>Results:</H3>
Thus, Kalman filter is implemented to predict the next position and   velocity in Python
