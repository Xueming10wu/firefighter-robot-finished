#include "MultCode.h"


MultCode::MultCode()
{
    from = 0;
    to = 0;
    num = 0;
    type = 0;
    data = 0;

    rxdata = new char[sumSize];
    txdata = new char[sumSize];
    tempBuffer = new char[sumSize];
    //rxdata = ( char* )malloc( sizeof(char) * sumSize );
    //txdata = ( char* )malloc( sizeof(char) * sumSize );
    //tempBuffer = ( char* )malloc( sizeof(char) * sumSize );
    //cout << "协议转换器初始化完成" << endl;
}

/*
MultCode::~MultCode()
{
    //dtor
}
*/

//输入字段，返回合并后的字段数组
char* MultCode::encode(int from, int to, int num, int type, int data)
{
    char * _from = judgement(from, fromeSize);
    char * _to = judgement(to, toSize);
    char * _num = judgement(num, numSize);
    char * _type = judgement(type, typeSize);
    char * _data = judgement(data, dataSize);

    int position = 0;
    for(int i = 0; i < fromeSize; i ++)
    {
        txdata[position] = _from[i];
        position ++;
    }

    for(int i = 0; i < toSize; i ++)
    {
        txdata[position] = _to[i];
        position ++;
    }

    for(int i = 0; i < numSize; i ++)
    {
        txdata[position] = _num[i];
        position ++;
    }

    for(int i = 0; i < typeSize; i ++)
    {
        txdata[position] = _type[i];
        position ++;
    }

    for(int i = 0; i < dataSize; i ++)
    {
        txdata[position] = _data[i];
        position ++;
    }
    delete _from;
    delete _to;
    delete _num;
    delete _type;
    delete _data;

    return txdata;
}


//输入数组，将不同字段放入不同的字段中。通过get～()方法即可获得数据
void MultCode::decode(char * data)
{
    for(int i = 0 ; i < sumSize; i ++)
    {
        rxdata[i] = data[i];
    }
    for(int i = 0 ; i < sumSize; i ++)
    {
        tempBuffer[i] = data[i];
    }

    int position = 0;


    char * _from = cut(position, position + fromeSize);
    setFrom
    (
        char_ToInt
        (
            _from,
            fromeSize
        )
    );
    position += fromeSize;
    delete _from;


    char * _to = cut(position, position + toSize);
    setTo
    (
        char_ToInt
        (
            _to,
            toSize
        )
    );
    position += toSize;
    delete _to;

    char * _num = cut(position, position + numSize);
    setNum
    (
        char_ToInt
        (
            _num,
            numSize
        )
    );
    position += numSize;
    delete _num;

    char * _type = cut(position, position + typeSize);
    setType
    (
        char_ToInt
        (
            _type,
            typeSize
        )
    );
    position += typeSize;
    delete _type;


    char * _data = cut(position, position + dataSize);
    setData
    (
        char_ToInt
        (
            _data,
            dataSize
        )
    );
    position += dataSize;
    delete _data;
}


void MultCode::setFrom( int data ){this->from = data;}
void MultCode::setTo( int data ){this->to = data;}
void MultCode::setNum( int data ){this->num = data;}
void MultCode::setType( int data ){this->type = data;}
void MultCode::setData( int data ){this->data = data;}

int MultCode::getFrom() const{ return from;}
int MultCode::getTo() const{ return to;}
int MultCode::getNum() const{ return num;}
int MultCode::getType() const{ return type;}
int MultCode::getData() const{ return data;}

int MultCode::getFromeSize() const{return fromeSize;}
int MultCode::getToSize() const{return toSize;}
int MultCode::getNumSize() const{return numSize;}
int MultCode::getTypeSize() const{return typeSize;}
int MultCode::getDataSize() const{return dataSize;}
int MultCode::getSumSize() const{return sumSize;}
char* MultCode::getTxData() const{return txdata;};
char* MultCode::getRxData() const{return rxdata;};

int MultCode::char_ToInt(char* str, int length) const
{
    int num = 0;
    char char_temp;
    int int_temp;
    for(int i = 0 ; i < length; i ++)
    {
        char_temp = str[i];
        int_temp = (int)char_temp -48;
        num += int_temp;
        num *= 10;
    }
    num = num / 10;
    return num;
}

//数据 位数
char* MultCode::judgement(int data, int bit)
{
    //char *s = (char *)malloc(bit * sizeof(char));
    char *s = new char[bit];

    for( int i = 0 ; i < bit ; i++)
    {
        s[i] = '0';
    }

    int remind = 0;

    for(int i = 0; data > 0; i ++)
    {
        remind = data % 10;
        data = data / 10;

        s[bit - i - 1] = (char)(remind + 48);
    }
    return s;
}

//起始位置 结束位置
char* MultCode::cut(int begin, int end)
{
    //char *s = (char *)malloc((end - begin + 1) * sizeof(char));
    char *s = new char[end - begin + 1];
    int j = 0;
    for(int i = begin; i < end; i ++)
    {
        s[j] = this->tempBuffer[i];
        j ++;
    }
    return s;
}
