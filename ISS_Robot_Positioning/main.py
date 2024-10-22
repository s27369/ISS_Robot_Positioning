import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
from datetime import datetime
data_path = 'data'
charts_path = 'charts'
def plot_results(x, y, model, save=False):
    plt.figure(figsize=(8, 6))
    plt.scatter(x, y, color='blue', label='Raw data')

    #predictions
    x_range = np.linspace(x.min(), x.max(), 100).reshape(-1, 1)#returns evenly spaced numbers over a specified interval.
    y_pred = model.predict(x_range)
    plt.plot(x_range, y_pred, color='red', linewidth=2, label='Regression line')

    plt.title('PWM vs Speed (v)')
    plt.xlabel('PWM')
    plt.ylabel('Speed (v)')
    plt.legend()
    plt.grid(True)


    if save:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.savefig('{}/plot_{}.png'.format(charts_path, timestamp), format='png')

    plt.show()

if __name__=="__main__":
    data = pd.read_csv('{}/data_test.csv'.format(data_path), dtype={'PWM': int, 'v': float},sep=',')
    # data = pd.read_csv('{}/data_test.csv'.format(data_path), dtype={'PWM': int, 'v': float},sep=',')

    #chatGPT
    X = data[['PWM']]
    y = data['v']
    # chatGPT

    # print(X)
    # print()
    # print(y)

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    model = LinearRegression()
    model.fit(X_train, y_train)

    def predict_speed(pwm_value):
        #chatGPT
        return model.predict(np.array(pwm_value).reshape(-1, 1))
        #chatGPT

    y_pred = model.predict(X_test)

    # chatGPT
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    print(f'Mean Squared Error (MSE): {mse}')
    print(f'R-squared: {r2}')
    # chatGPT

    plot_results(data["PWM"], data["v"], model, True)


