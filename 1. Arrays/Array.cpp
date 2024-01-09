#include<iostream>
using namespace std;

class Array{
    int *items;
    int count;
    int maxSize;

    public:
    Array(int length){
        count=0;
        maxSize=length;
        items=new int[maxSize];
    }

    void insert(int item){
        if(count!=maxSize){
            items[count++]=item;
        }else{
            cout<<"Array is full"<<endl;
        }
    }
    void insertAt(int item,int index){
        if(index<0||index>count){
            cout<<"Wrong index provided."<<endl;
            return;
        }
        for(int i=count-1;i>=index;--i){
            items[i+1]=items[i];
        }
        items[index]=item;
        ++count;
    }
    void reverse(){
        int *revItems=new int[maxSize];

        for(int i=0;i<count;++i){
            revItems[i]=items[count-i-1];
        }
        delete[] items;
        items=revItems;
    }
    int max(){
        int max=0;
        for(int i=0;i<count;++i){
            int item=items[i]
            if(item>max){
                max=item;
            }
        }
        return max;
    }
    void removeAt(int index){
        if(index<0||index>count){
            cout<<"Wrong index provided."<<endl;
            return;
        }

        for(int i=index;i<count;++i){
            items[i]=items[i+1];
        }
        --count;
    }
    int indexOf(int item){
        for(int i=0;i<count;++i){
            if(items[i]==item){
                return i;
            }
        }
        return -1;
    }
    void print(){
        for(int i=0;i<count;++i){
            cout<<items[i]<<", ";
        }
        cout<<endl;
    }
}