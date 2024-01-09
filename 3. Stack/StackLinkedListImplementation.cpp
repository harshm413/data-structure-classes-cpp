#include <iostream>
using namespace std;

class Stack {
    struct Node {
        int data;
        Node* next;

        Node(int val) {
			data = val;
			next = nullptr;
		}
    };

    Node* top;

public:
    Stack() {
        top = nullptr;
    }

    void push(int item) {
        Node* temp = new Node(item);
        temp->next = top;
        top = temp;
    }

    void pop() {
        if (top == nullptr)
            cout << "Stack is empty.";
        else {
            Node* temp = top;
            top = top->next;
            delete temp;
        }
    }

    int peek() {
        if (top == nullptr) {
            cout << "Stack is empty.";
            return -1;
        } else {
            return top->data;
        }
    }

    void display() {
        Node* temp = top;
        cout << "\n";
        while (temp != nullptr) {
            cout << temp->data << " ";
            temp = temp->next;
        }
    }
};
