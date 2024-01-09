#include<iostream>
using namespace std;

class AVLTree {
private:
    struct Node {
        int value;
        int height;
        Node* leftChild;
        Node* rightChild;

        Node(int data){
            value=data;
            height=0;
            leftChild=rightChild=nullptr;
        }
    };

    Node* root;

public:
    AVLTree(){
        root=nullptr;
    }

    void insert(int value) {
        root = insert(root, value);
    }

private:
    Node* insert(Node* root, int value) {
        if (root == nullptr) {
            return new Node(value);
        }

        if (value < root->value) {
            root->leftChild = insert(root->leftChild, value);
        }
        else {
            root->rightChild = insert(root->rightChild, value);
        }

        setHeight(root);

        return balance(root);
    }

    Node* balance(Node* root) {
        if (isLeftHeavy(root)) {
            if (balanceFactor(root->leftChild) < 0) {
                root->leftChild = rotateLeft(root->leftChild);
            }
            return rotateRight(root);
        }
        else if (isRightHeavy(root)) {
            if (balanceFactor(root->rightChild) > 0) {
                root->rightChild = rotateRight(root->rightChild);
            }
            return rotateLeft(root);
        }
        return root;
    }

    Node* rotateLeft(Node* root) {
        Node* newRoot = root->rightChild;

        root->rightChild = newRoot->leftChild;
        newRoot->leftChild = root;

        setHeight(root);
        setHeight(newRoot);

        return newRoot;
    }

    Node* rotateRight(Node* root) {
        Node* newRoot = root->leftChild;

        root->leftChild = newRoot->rightChild;
        newRoot->rightChild = root;

        setHeight(root);
        setHeight(newRoot);

        return newRoot;
    }

    void setHeight(Node* node) {
        if (node == nullptr) return;
        node->height = 1 + max(height(node->leftChild), height(node->rightChild));
    }

    bool isLeftHeavy(Node* node) {
        return balanceFactor(node) > 1;
    }

    bool isRightHeavy(Node* node) {
        return balanceFactor(node) < -1;
    }

    int balanceFactor(Node* node) {
        return (node == nullptr) ? 0 : height(node->leftChild) - height(node->rightChild);
    }

    int height(Node* node) {
        return (node == nullptr) ? -1 : node->height;
    }
};
