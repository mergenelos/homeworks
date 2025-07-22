import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Controller parameters 
k11 = 10.0
k12 = 7.5
k21 = 5.0
k22 = 1.0
a1 = 1.5
a2 = 1.5
r1 = 10.0
r2 = 10.0
sigma11 = 0.05
sigma12 = 0.05
sigma21 = 0.05
sigma22 = 0.05
h = 95.0 / 101.0

# Quantizer parameters 
v_min = 0.1
delta = 0.1
rho = (1 - delta) / (1 + delta)

# Simulation parameters
t_span = [0, 30]
t_eval = np.linspace(t_span[0], t_span[1], 1000)
# Initial conditions 
z1_0 = 0.5
z2_0 = 0.5
theta1_0 = 0.0
theta2_0 = 0.0
initial_conditions = [z1_0, z2_0, theta1_0, theta2_0]


# Fuzzy logic system 1 (1 input, 7 rules)
fls1_centers = np.linspace(-1.5, 1.5, 7)
fls1_width = 1.0
# Fuzzy logic system 2 (1 input, 7 rules)
fls2_centers = np.linspace(-3.0, 3.0, 7)
fls2_width = 1.0

def gaussian_basis(x, centers, width):
    #Calculate the Gaussian basis function
    return np.exp(-(x - centers)**2 / width**2)

# Implements the logic from equation (6)
class HystereticQuantizer:
    def __init__(self, v_min, delta, rho):
        self.v_min = v_min
        self.delta = delta
        self.rho = rho
        self.q_prev = 0
        self.v_prev = 0

    def quantize(self, v):
        v_abs = np.abs(v)
        v_dot = v - self.v_prev # Discrete-time derivative approximation
        
        # Direct implementation of the conditions in eq. (6)
        if v_abs <= self.v_min:
            if v_abs <= self.v_min / (1 + self.delta):
                 q_current = 0
            else:
                 q_current = self.v_min * np.sign(v)
        else:
            i = np.floor(np.log(v_abs / self.v_min) / np.log(1/self.rho)) + 1
            v_i = self.rho**(1 - i) * self.v_min
            
            if v_abs > v_i * (1 + self.delta):
                 q_current = v_i * (1 + self.delta) * np.sign(v)
            elif v_abs > v_i:
                 q_current = v_i * np.sign(v)
            else:
                 q_current = self.q_prev

        self.q_prev = q_current
        self.v_prev = v
        return q_current

# System Dynamics and Control Law
def power(val, p):
    return np.sign(val) * (np.abs(val)**p)

# Instantiate the quantizer
quantizer = HystereticQuantizer(v_min, delta, rho)

def system_model(t, y):
    # Defines the complete ODE system including the plant and controller.
    z1, z2, theta1, theta2 = y

    # Reference signal and its derivative 
    y_r = np.sin(t)
    y_r_dot = np.cos(t)

    # Error coordinates 
    eta1 = z1 - y_r
    
    # Virtual Controller and Adaptive Law
    X1 = np.array([z1]) 
    S1 = gaussian_basis(X1, fls1_centers, fls1_width)
    S1_T_S1 = S1.T @ S1
    
    # Virtual controller alpha1
    alpha1 = -(k11 + 0.5) * eta1 - k12 * power(eta1, 2 * h - 1) \
             - (eta1 / (2 * a1**2)) * theta1 * S1_T_S1

    # Adaptive law for theta1
    theta1_dot = (r1 / (2 * a1**2)) * eta1**2 * S1_T_S1 \
                 - sigma11 * theta1 - sigma12 * power(theta1, 2 * h - 1)
    
    # Ensure theta remains non-negative as per Lemma 4 
    if theta1 < 0:
        theta1 = 0
        if theta1_dot < 0:
            theta1_dot = 0

    eta2 = z2 - alpha1
    
    # Actual Controller and Adaptive Law
    X2 = np.array([z2])
    S2 = gaussian_basis(X2, fls2_centers, fls2_width)
    S2_T_S2 = S2.T @ S2

    # Pre-quantized control input 'v'
    v_control_term = (k21 + 0.5) * eta2 + k22 * power(eta2, 2 * h - 1) + \
                     (eta2 * theta2 / (2 * a2**2)) * S2_T_S2
    v = (-1 / (1 - delta)) * v_control_term - v_min * np.sign(eta2)

    # Quantized control input 'u'
    u = quantizer.quantize(v)
    
    # Adaptive law for theta2
    theta2_dot = (r2 / (2 * a2**2)) * eta2**2 * S2_T_S2 \
                 - sigma21 * theta2 - sigma22 * power(theta2, 2 * h - 1)

    if theta2 < 0:
        theta2 = 0
        if theta2_dot < 0:
            theta2_dot = 0

    # Plant dynamics 
    z1_dot = 0.5 * z1**3 + (1 + np.sin(z1)**2) * z2
    z2_dot = z2**2 * np.cos(z1) * np.sin(z2) + 2 * u

    return [z1_dot, z2_dot, theta1_dot, theta2_dot]

# Solve the ODE System
solution = solve_ivp(system_model, t_span, initial_conditions, t_eval=t_eval, method='RK45')

# Post-process to get control signal for plotting
z1_sol, z2_sol, theta1_sol, theta2_sol = solution.y
y_r_sol = np.sin(t_eval)
eta1_sol = z1_sol - y_r_sol

# Recalculate alpha1
alpha1_sol = np.zeros_like(t_eval)
for i in range(len(t_eval)):
    S1 = gaussian_basis(np.array([z1_sol[i]]), fls1_centers, fls1_width)
    S1_T_S1 = S1.T @ S1
    alpha1_sol[i] = -(k11 + 0.5) * eta1_sol[i] - k12 * power(eta1_sol[i], 2 * h - 1) \
                    - (eta1_sol[i] / (2 * a1**2)) * theta1_sol[i] * S1_T_S1

eta2_sol = z2_sol - alpha1_sol

# Recalculate quantized input u
u_sol = np.zeros_like(t_eval)
plot_quantizer = HystereticQuantizer(v_min, delta, rho)
for i in range(len(t_eval)):
    S2 = gaussian_basis(np.array([z2_sol[i]]), fls2_centers, fls2_width)
    S2_T_S2 = S2.T @ S2
    v_control_term = (k21 + 0.5) * eta2_sol[i] + k22 * power(eta2_sol[i], 2 * h - 1) + \
                     (eta2_sol[i] * theta2_sol[i] / (2 * a2**2)) * S2_T_S2
    v = (-1 / (1 - delta)) * v_control_term - v_min * np.sign(eta2_sol[i])
    u_sol[i] = plot_quantizer.quantize(v)


# Plotting the Results
plt.style.use('default')

# Figure 1: y and yd 
plt.figure(figsize=(8, 6))
plt.plot(t_eval, z1_sol, 'r-', label='y (System Output)')
plt.plot(t_eval, y_r_sol, 'k--', label='yd (Reference Signal)')
plt.title('Figure 1: System Output vs. Reference Signal')
plt.xlabel('Time(sec)')
plt.ylabel('')
plt.ylim(-1.5, 1.5)
plt.grid(True)
plt.legend()
plt.show()

# Figure 2: State z2 
plt.figure(figsize=(8, 6))
plt.plot(t_eval, z2_sol, 'b-')
plt.title('Figure 2: State Variable z2')
plt.xlabel('Time(sec)')
plt.ylabel('z2')
plt.grid(True)
plt.show()

# Figure 3: Adaptive Parameters 
plt.figure(figsize=(8, 6))
plt.plot(t_eval, theta1_sol, 'r-', label=r'$\theta_1$')
plt.plot(t_eval, theta2_sol, 'k--', label=r'$\theta_2$')
plt.title('Figure 3: Adaptive Parameters')
plt.xlabel('Time(sec)')
plt.ylabel('')
plt.legend()
plt.grid(True)
plt.show()

# Figure 4: Quantizer Output 
plt.figure(figsize=(8, 6))
plt.plot(t_eval, u_sol, 'b-')
plt.title('Figure 4: Quantizer Output q(u)')
plt.xlabel('Time(sec)')
plt.ylabel('q(u)')
plt.grid(True)
plt.show()