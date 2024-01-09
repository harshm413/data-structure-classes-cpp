#include<iostream>
#include<vector>
using namespace std;
class BinarySearchTree{
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
    Node* deleteNode(Node* root, int value) {
        if (root == nullptr) {
            return nullptr;
        }

        if (value < root->value) {
            root->left = deleteNode(root->left, value);
        } else if (value > root->value) {
            root->right = deleteNode(root->right, value);
        } else {
            // The node to be deleted is a leaf node.
            if (root->left == nullptr && root->right == nullptr) {
                delete root;
                return nullptr;
            }

            // The node to be deleted has one child node.
            if (root->left == nullptr) {
                Node* temp = root->right;
                delete root;
                return temp;
            } else if (root->right == nullptr) {
                Node* temp = root->left;
                delete root;
                return temp;
            }

            // The node to be deleted has two child nodes.
            // Find the inorder successor of the node to be deleted.
            Node* successor = root->right;
            while (successor->left != nullptr) {
                successor = successor->left;
            }

            // Copy the value of the inorder successor to the node to be deleted.
            root->value = successor->value;

            // Delete the inorder successor.
            root->right = deleteNode(root->right, successor->value);
        }

        return root;
    }
    bool equals(Node *first, Node *second){
        if(first==nullptr && second==nullptr)
            return true;
        if(first!=nullptr && second!=nullptr)
            return first->value == second->value
                && equals(first->left,second->left)
                && equals(first->right,second->right);
        return false;
    }
    public:
    BinarySearchTree(){
        root=nullptr;
    }
    void insert(int val){
        if(root==nullptr){
            root=new Node(val);
            return;
        }

        Node *current=root;
        while(true){
            if(val<current->value){
                if(current->left==nullptr){
                    current->left=new Node(val);
                    break;
                }
                current=current->left;
            }
            else if(val>current->value){
                if(current->right==nullptr){
                    current->right=new Node(val);
                    break;
                }
                current=current->right;
            }
        }
    }
    bool find(int val){
        Node *current=root;
        while(current!=nullptr){
            if(val<current->value)
                current=current->left;
            else if(val>current->value)
                current=current->right;
            else
                return true;
        }
        return false;
    }
    void preOrder(){
        preOrder(root);
        cout<<endl;
    }
    int height(){
        return height(root);
    }
    void printNodeAtDistance(int dist){
        printNodeAtDistance(root,dist);
    }
    void levelOrder(){
        for(int i=0;i<=height();++i){
            printNodeAtDistance(i);
            cout<<endl;
        }
    }
    void deleteNode(int val){
        root=deleteNode(root,val);
    }
    bool equals(BinarySearchTree other){
        return equals(root,other.root);
    }
};