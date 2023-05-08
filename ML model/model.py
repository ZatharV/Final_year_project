import pandas as pd
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split

# Load the data
data = pd.read_csv('data.csv')

# Convert timestamp to datetime object
data['TIMESTAMP'] = pd.to_datetime(data['TIMESTAMP'], format='%H:%M:%S')

# Split the data into training and test sets
train_data, test_data = train_test_split(data, test_size=0.1, random_state=42)

print("train data", train_data)
print("train data", test_data)

# Define the features and target variables
features = ['TIMESTAMP']
target_vars = ['TEMPERATURE', 'HUMIDITY']

# Train a random forest model for temperature prediction
rf_temp = RandomForestRegressor(n_estimators=100, random_state=42)
rf_temp.fit(train_data[features], train_data['TEMPERATURE'])

# Predict temperature values for the test data
temp_pred = rf_temp.predict(test_data[features])

# Evaluate the model using mean squared error
temp_mse = mean_squared_error(test_data['TEMPERATURE'], temp_pred)
print(f'Temperature MSE: {temp_mse}')

# Train a random forest model for humidity prediction
rf_humi = RandomForestRegressor(n_estimators=100, random_state=42)
rf_humi.fit(train_data[features], train_data['HUMIDITY'])

# Predict humidity values for the test data
humi_pred = rf_humi.predict(test_data[features])

# Evaluate the model using mean squared error
humi_mse = mean_squared_error(test_data['HUMIDITY'], humi_pred)
print(f'Humidity MSE: {humi_mse}')

# Plot actual vs predicted temperature values
plt.figure()
plt.scatter(test_data['TIMESTAMP'], test_data['TEMPERATURE'], color='blue', label='Actual')
plt.scatter(test_data['TIMESTAMP'], temp_pred, color='red', label='Predicted')
plt.xlabel('Timestamp')
plt.ylabel('Temperature')
plt.title('Actual vs Predicted Temperature')
plt.legend()
plt.show()

# Plot actual vs predicted humidity values
plt.figure()
plt.scatter(test_data['TIMESTAMP'], test_data['HUMIDITY'], color='blue', label='Actual')
plt.scatter(test_data['TIMESTAMP'], humi_pred, color='red', label='Predicted')
plt.xlabel('Timestamp')
plt.ylabel('Humidity')
plt.title('Actual vs Predicted Humidity')
plt.legend()
plt.show()

# new_data = pd.read_csv('new_data.csv')

# # Convert timestamp to datetime object
# new_data['TIMESTAMP'] = pd.to_datetime(new_data['TIMESTAMP'], format='%H:%M:%S')

# # Predict temperature and humidity values for the new data
# new_temp_pred = rf_temp.predict(new_data[features])
# new_humi_pred = rf_humi.predict(new_data[features])

# # Calculate the error rate compared to the predicted data
# temp_error_rate = abs(new_temp_pred - temp_pred).mean() / temp_pred.mean()
# humi_error_rate = abs(new_humi_pred - humi_pred).mean() / humi_pred.mean()

# print(f'Temperature error rate: {temp_error_rate}')
# print(f'Humidity error rate: {humi_error_rate}')
