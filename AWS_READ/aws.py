import boto3
import pandas as pd
import time

aws_access_key_id = 'AKIATYVWQFBJREGXXM72'
aws_secret_access_key = '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'
aws_region = 'ap-south-1'

dynamodb = boto3.client('dynamodb',
                        aws_access_key_id=aws_access_key_id,
                        aws_secret_access_key=aws_secret_access_key,
                        region_name=aws_region)

table_name = 'taledata2'
scan_params = {
    'TableName': table_name,
}

while(1):
    response = dynamodb.scan(**scan_params)
    items = response['Items']

    df = pd.DataFrame(items)

    print(df.head())
    df.to_csv('D:/nipun prev laptop/final_year/Project scripts/AWS_READ/my_data.csv', index=True)
    time.sleep(10)

