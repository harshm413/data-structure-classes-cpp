#include<iostream>
#include<vector>
using namespace std;
class BinaryTree{
    struct Node{
        int value;
        Node *left,*right;
        Node(int val){
            value=val;
            left=right=nullptr;
        }
    };
    Node *root;
    Node* buildBinaryTree(Node *root){
        int value;
        cin>>value;
        if(value == -1){
            return nullptr;
        }
        root = new Node(value);
        root->left = buildBinaryTree(root->left);
        root->right = buildBinaryTree(root->right);

        return root;
    }
    int height(Node *root){
        if(root==nullptr){
            return -1;
        }
        if(root->left==nullptr && root->right==nullptr){
            return 0;
        }
        return 1+max(height(root->left),height(root->right));
    }
    void printNodeAtDistance(Node *root, int dist){
        if(root==nullptr) return;
        if(dist==0){
            cout<<root->value<<" ";
        }
        printNodeAtDistance(root->left,dist-1);
        printNodeAtDistance(root->right,dist-1);
    }
    public:
    BinaryTree(){
        root=nullptr;
    }
    void printNodeAtDistance(int dist){
        printNodeAtDistance(root,dist);
    }
    int height(){
        return height(root);
    }
    void buildBinaryTree(){
        root = buildBinaryTree(root);
    }
    void levelOrder(){
        for(int i=0;i<=height();++i){
            printNodeAtDistance(i);
            cout<<endl;
        }
    }
};