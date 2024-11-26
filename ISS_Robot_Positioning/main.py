import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
from datetime import datetime

data_path = 'data'
charts_path = 'charts'

def print_model_params(model):
    #GPT
    a = model.coef_[0]  # Slope (coefficient)
    b = model.intercept_  # Intercept
    print(f"Linear regression model: y = {a:.4f} * x + {b:.4f}")
    return a, b
    # GPT
def make_plot(model_dict, data, save=False):

    plt.figure(figsize=(8, 6))
    colors = {
        "motor_l":["cyan", "blue"],
        "motor_r":["LawnGreen", "green"],
    }
    #scatter data
    for k, v in model_dict.items():
        plt.scatter(data["PWM"], data[k], color=colors[k][0], label=f'{k} raw data ', alpha=0.8)

    x_range = np.linspace(data["PWM"].min(), data["PWM"].max(), 100).reshape(-1, 1)
    for k, v in model_dict.items():
        plt.plot(x_range, v.predict(x_range), color=colors[k][1], linewidth=2, label=f'{k} regression line')

    plt.title('PWM vs Measurements for each motor')
    plt.xlabel('PWM')
    plt.ylabel('Measurements')
    plt.legend()
    plt.grid(True)
    if save:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.savefig(f'{charts_path}/plot_{timestamp}.png', format='png')

    plt.show()

def make_model_for_column(col_name, dataset):
    X_train, X_test, y_train, y_test = train_test_split(data[['PWM']], data[col_name], test_size=0.2, random_state=42)
    model = LinearRegression()
    model.fit(X_train, y_train)
    y_pred = model.predict(X_test)

    # Evaluate the model
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    print(f"Decision column: {col_name}")
    print(f'Mean Squared Error (MSE): {mse}')
    print(f'R-squared: {r2}')
    print(print_model_params(model))
    return model


if __name__ == "__main__":
    data = pd.read_csv(f'{data_path}/data.csv', sep=',')

    models = {
        "motor_l": make_model_for_column("motor_l", data),
        "motor_r": make_model_for_column("motor_r", data)
    }

    make_plot(models, data, True)

