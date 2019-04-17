#ifndef MULTCODE_H
#define MULTCODE_H

class MultCode
{
    public:
        MultCode();
        //virtual ~MultCode();

        //输入字段，返回合并后的字段数组
        char* encode(int from, int to, int num, int type, int data);

        //输入数组，将不同字段放入不同的字段中。通过get～()方法即可获得数据
        void decode(char * data);

        void setFrom( int data );
        void setTo( int data );
        void setNum( int data );
        void setType( int data );
        void setData( int data );

        int getFrom() const;
        int getTo() const;
        int getNum() const;
        int getType() const;
        int getData() const;

	int getFromeSize() const;
        int getToSize() const;
        int getNumSize() const;
        int getTypeSize() const;
        int getDataSize() const;
	int getSumSize() const;
        char* getTxData() const;
        char* getRxData() const;

        int char_ToInt(char* str, int length) const;


    private:
        //起始位置 结束位置
        char* cut(int begin, int end);

        //数据 位数
        char* judgement(int data, int bit);

        //数据段描述
        //发送方
        int from;
        //接收方
        int to;
        //设备编号
        int num;
        //某个设备的某种数据类型编号
        int type;
        //数据
        int data;

        //各个字段的长度
        const int fromeSize = 2;
        const int toSize = 2;
        const int numSize = 8;
        const int typeSize = 4;
        const int dataSize = 8;
        const int sumSize = fromeSize + toSize + numSize + typeSize + dataSize;

        //保存传入数据
        char *rxdata;
        //保存传出数据
        char *txdata;
        //缓存区
        char *tempBuffer;
};

#endif // MULTCODE_H
