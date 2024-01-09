#include<iostream>
using namespace std;

class LinkedList{
    struct Node{
        int value;
        Node *next;

        Node(int val){
            value=val;
            next=nullptr;
        }
    };

    Node *head,*tail;
    int size;

    bool isEmpty(){
        return head==nullptr;
    }
    Node* getPrevious(Node *node){
        Node *current=head;
        while(current!=nullptr){
            if(current->next==node){
                return current;
            }
            current=current->next;
        }
        return nullptr;
    }

    public:
    LinkedList(){
        head=tail=nullptr;
        size=0;
    }

    void addLast(int item){
        Node *node=new Node(item);

        if(isEmpty()){
            head=tail=node;
        }else{
            tail->next=node;
            tail=node;
        }
        ++size;
    }
    void addFirst(int item){
        Node *node=new Node(item);

        if(isEmpty()){
            head=tail=node;
        }else{
            node->next=head;
            head=node;
        }
        ++size;
    }
    int indexOf(int item){
        int index=0;
        Node *current=head;
        while(current!=nullptr){
            if(current->value==item){
                return index;
            }
            current=current->next;
            ++index;
        }
        return -1;
    }
    bool contains(int item){
        return indexOf(item)!=-1;
    }
    void removeFirst(){
        if(isEmpty()){
            cout<<"List is empty"<<endl;
        }

        if(head==tail){
            head=tail=nullptr;
        }else{
            Node *temp=head;
            head=head->next;
            delete temp;
        }
        --size;
    }
    void removeLast(){
        if(isEmpty()){
            cout<<"List is Empty."<<endl;
        }else{
            Node *previous=getPrevious(tail);
            delete tail;
            tail=previous;
            tail->next=nullptr;
        }
        --size;
    }
    int getSize(){
        return size;
    }
    void reverse(){
        if(isEmpty()) return;

        Node *previous=head;
        Node *current=head->next;

        while(current!=nullptr){
            Node *next=current->next;
            current->next=previous;
            previous=current;
            current=next;
        }
        tail=head;
        tail->next=nullptr;
        head=previous;
    }
}