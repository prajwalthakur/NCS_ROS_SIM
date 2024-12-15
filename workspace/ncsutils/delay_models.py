from dataclasses import dataclass
from scipy.stats import truncnorm
import numpy as np
# import matplotlib.pyplot as plt
SEED_VALUE = 113 
np.random.seed(SEED_VALUE)
rng = np.random.RandomState(SEED_VALUE) 
import pdb
@dataclass
class turncated_normal_delay:
  low:np.float32
  high: np.float32
  mu: np.float32
  sigma: np.float32
  def __init__(self,mu:np.float32 , sigma:np.float32  , low:np.float32 , high: np.float32):
    self.low = low
    self.high = high
    self.mu = mu
    self.sigma = sigma
    self.a  = (self.low - self.mu)/self.sigma
    self.b =   (self.high - self.mu)/self.sigma
    
  def sample_delay(self,size:np.ndarray):
      samples = truncnorm.rvs(self.a, self.b, loc=self.mu, scale=self.sigma, size=size,random_state=rng)
      return samples
    
@dataclass
class constant_delay:
  delay : np.float32
  
  def __init_(self, delay:np.float32):
    self.delay = delay
    
  def sample_delay(self,size:np.ndarray=0):
      samples = np.ones(size)*self.delay
      return samples


@dataclass
class sawtooth_sine_delay:
  time_steps: np.float32
  base_latency: np.float32
  saw_amplitude: np.float32
  saw_period: np.float32
  sine_amplitude: np.float32
  sine_period: np.float32
  def __init__(self,time_steps: int, base_latency: float, saw_amplitude: float, saw_period: float,
                 sine_amplitude: float, sine_period: float,delay_model:object):
    self.time_steps = time_steps
    self.base_latency = base_latency
    self.saw_amplitude = saw_amplitude
    self.saw_period = saw_period
    self.sine_amplitude = sine_amplitude
    self.sine_period = sine_period
    self.delay_model = delay_model
    self.time = np.arange(self.time_steps) / 60  # Time in minutes

  def sample_delay(self,times,size:np.ndarray=0):
      sine_sample = self.sine_amplitude * np.sin(2 * np.pi * times / self.sine_period)
      sawtooth_sample = self.saw_amplitude * (1 - (times % self.saw_period) / self.saw_period)
      true_latency = self.base_latency + sawtooth_sample + sine_sample
      #samples = np.ones(size)*self.delay
      observed_latency = true_latency + self.delay_model.sample_delay(len(times))
      return observed_latency


  # def plot_delay_profile(self, times: np.ndarray):
  #     true_latency, observed_latency = self.generate_delay_at_time(times)
      
  #     # Plotting results
  #     plt.figure(figsize=(14, 6))
  #     plt.plot(times, true_latency, label="True Latency", linestyle="--", color="black", linewidth=1.5)
  #     plt.plot(times, observed_latency, label="Observed Latency (Noisy)", color="red", alpha=0.7)
  #     plt.xlabel("Time (seconds)")
  #     plt.ylabel("Latency (ms)")
  #     plt.title("Delay Profile at Given Time Instances")
  #     plt.legend()
  #     plt.show()

@dataclass
class sawtooth_delay:
  time_steps: np.float32
  base_latency: np.float32
  saw_amplitude: np.float32
  def __init__(self,time_steps: int, base_latency: float, saw_amplitude: float, saw_period: float,
                 sine_amplitude: float, sine_period: float,delay_model:object):
    self.time_steps = time_steps
    self.base_latency = base_latency
    self.saw_amplitude = saw_amplitude
    self.saw_period = saw_period
    self.sine_amplitude = sine_amplitude
    self.sine_period = sine_period
    self.delay_model = delay_model
    self.time = np.arange(self.time_steps) / 60  # Time in minutes

  def sample_delay(self,times,size:np.ndarray=0):
      sine_sample = self.sine_amplitude * np.sin(2 * np.pi * times / self.sine_period)
      sawtooth_sample = self.saw_amplitude * (1 - (times % self.saw_period) / self.saw_period)
      true_latency = self.base_latency + sawtooth_sample + sine_sample
      #samples = np.ones(size)*self.delay
      observed_latency = true_latency + self.delay_model.sample_delay(len(times))
      return observed_latency


  # def plot_delay_profile(self, times: np.ndarray):
  #     true_latency, observed_latency = self.generate_delay_at_time(times)
      
  #     # Plotting results
  #     plt.figure(figsize=(14, 6))
  #     plt.plot(times, true_latency, label="True Latency", linestyle="--", color="black", linewidth=1.5)
  #     plt.plot(times, observed_latency, label="Observed Latency (Noisy)", color="red", alpha=0.7)
  #     plt.xlabel("Time (seconds)")
  #     plt.ylabel("Latency (ms)")
  #     plt.title("Delay Profile at Given Time Instances")
  #     plt.legend()
  #     plt.show()



@dataclass
class moving_avg_class():
  def __init__(self):
    self.itr = 0
    self.current_avg = 0.0
  def compute_avg(self,current_measurement):
    self.itr+=1
    self.current_avg =  self.current_avg  +  (current_measurement-self.current_avg)/self.itr
    return self.current_avg
    
    

# # Example Usage:
# # Create a delay model (either truncated normal or constant)
# delay_model = TruncatedNormalDelay(mu=0.35, sigma=0.2, low=0.0, high=0.7)
# # Alternatively, use constant delay model:
# # delay_model = ConstantDelay(delay=300)

# # Define the specific time instances you want to sample the delay for (in seconds)
# time_instances = np.array([0, 0.3, 0.6, 0.9, 1.2])  # Time in seconds

# # Initialize and simulate the delay profile
# simulator = DelaySimulation(base_latency=300, saw_amplitude=200, saw_period=120,
#                             sine_amplitude=100, sine_period=60, delay_model=delay_model)

# # Plot the delay profile for these specific time instances
# simulator.plot_delay_profile(time_instances)

# # Print the true and observed latency at the specified time instances
# true_latency, observed_latency = simulator.generate_delay_at_time(time_instances)
# print("True Latency:", true_latency)
# print("Observed Latency:", observed_latency)
