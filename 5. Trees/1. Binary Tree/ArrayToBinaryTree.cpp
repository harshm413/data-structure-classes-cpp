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
    int max(int a,int b){
        if(a>=b) return a;
        else return b;
    }
    Node* insertLevelOrder(vector<int> &array, int index){
        Node *root=nullptr;
        if(index < array.size()){
            root=new Node(array[index]);
            root->left=insertLevelOrder(array,2*index+1);
            root->right=insertLevelOrder(array,2*index+2);
        }
        return root;
    }
    void preOrder(Node *root){
        if(root==nullptr)
            return;
    
        cout<<root->value<<", ";
        preOrder(root->left);
        preOrder(root->right);
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
    bool isBinarySearchTree(Node *root, int min, int max){
        if(root==nullptr){
            return true;
        }
        if(root->value<min || root->value>max){
            return false;
        }
        return isBinarySearchTree(root->left,min,root->value-1)
            && isBinarySearchTree(root->right,root->value+1,max);
    }
    public:
    BinaryTree(){
        root=nullptr;
    }
    void insertLevelOrder(vector<int> &array){
        root=insertLevelOrder(array, 0);
    }
    void preOrder(){
        preOrder(root);
        cout<<endl;
    }
    void printNodeAtDistance(int dist){
        printNodeAtDistance(root,dist);
    }
    int height(){
        return height(root);
    }
    void levelOrder(){
        for(int i=0;i<=height();++i){
            printNodeAtDistance(i);
            cout<<endl;
        }
    }
    bool isBinarySearchTree(){
        return isBinarySearchTree(root,INT_MIN,INT_MAX);
    }
};
