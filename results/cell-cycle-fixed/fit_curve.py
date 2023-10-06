# scipy version: 1.11.3
from scipy.optimize import curve_fit

def func(x, a, b, c):
    return a * x * x + b * x + c
    
relative_volume = [
101.9551018,
107.9604984,
115.3709399,
123.3029321,
131.2110512,
138.7731663,
145.8130126,
152.2481633,
158.0552476,
163.2468548,
167.856337,
171.9279316,
175.5104538,
178.6533687,
181.4044384,
183.8084021,
185.9063225,
187.7353554,
189.3287777,
190.7161688,
191.9236745,
194.6832014,
195.3745722,
195.9758402]

time_mins = range(0, 30*len(relative_volume), 30)

popt, pcov = curve_fit(func, time_mins, relative_volume)

print(popt)

