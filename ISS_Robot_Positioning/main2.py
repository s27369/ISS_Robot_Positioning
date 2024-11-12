import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
from datetime import datetime

# Paths (You can modify these paths according to your system)
data_path = 'data'
charts_path = 'charts'

def plot_results(x, y, model, save=False):
    plt.figure(figsize=(8, 6))
    plt.scatter(x, y, color='blue', label='Raw data')

    # Predictions
    x_range = np.linspace(x.min(), x.max(), 100).reshape(-1, 1)  # evenly spaced numbers over a specified interval
    y_pred = model.predict(x_range)
    plt.plot(x_range, y_pred, color='red', linewidth=2, label='Regression line')

    plt.title('PWM vs Speed (v)')
    plt.xlabel('PWM')
    plt.ylabel('Speed (v)')
    plt.legend()
    plt.grid(True)

    if save:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.savefig(f'{charts_path}/plot_{timestamp}.png', format='png')

    plt.show()

def print_model_params(model):
    # Extracting the model parameters (a and b)
    a = model.coef_[0]  # Slope (coefficient)
    b = model.intercept_  # Intercept
    print(f"Linear regression model: y = {a:.4f} * x + {b:.4f}")
    return a, b

if __name__ == "__main__":
    # Read the data from the CSV file
    data = pd.read_csv(f'{data_path}/data_test.csv', dtype={'PWM': int, 'v': float}, sep=',')

    # Define the input (PWM) and output (Speed) variables
    X = data[['PWM']]  # Input feature
    y = data['v']  # Output variable (speed)

    # Split the data into training and testing sets (80% train, 20% test)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # Initialize and train the Linear Regression model
    model = LinearRegression()
    model.fit(X_train, y_train)

    # Get model parameters (a and b)
    a, b = print_model_params(model)

    # Predict speed for the test set
    y_pred = model.predict(X_test)

    # Evaluate the model
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    print(f'Mean Squared Error (MSE): {mse}')
    print(f'R-squared: {r2}')

    # Plot the results
    plot_results(data["PWM"], data["v"], model, save=True)

    # Optionally, you can use the following to predict speed for any given PWM value
    def predict_speed(pwm_value):
        return model.predict(np.array(pwm_value).reshape(-1, 1))

    # Example: Predict speed for PWM value 100
    pwm_value = 100
    predicted_speed = predict_speed(pwm_value)
    print(f"Predicted speed for PWM = {pwm_value}: {predicted_speed[0]:.4f}")
