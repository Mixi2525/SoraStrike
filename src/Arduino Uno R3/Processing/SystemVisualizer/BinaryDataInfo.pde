class BinaryFloatDataInfo
{
    boolean expectingData; // データを受信中かどうかを示す汎用フラグ
    int numberOfDataToReceive; // 受信するデータの数
    int numberOfDataCurrentlyReceived; // 受信したデータの数
    String expectedDataType; // 期待するデータの種別

    BinaryFloatDataInfo(boolean expectingData9, int numberOfDataToReceive, int numberOfDataCurrentlyReceived, String expectedDataType)
    {
        setValue(expectingData, numberOfDataToReceive, numberOfDataCurrentlyReceived, expectedDataType);
    }

    void setValue(boolean expectingData, int numberOfDataToReceive, int numberOfDataCurrentlyReceived, String expectedDataType)
    {
        this.expectingData = expectingData;
        this.numberOfDataToReceive = numberOfDataToReceive;
        this.numberOfDataCurrentlyReceived = numberOfDataCurrentlyReceived;
        this.expectedDataType = expectedDataType;
    }

    void reset()
    {
        setValue(false, 0, 0, "");
    }
}