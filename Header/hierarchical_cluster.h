#ifndef HIERARCHICAL_CLUSTER_H
#define HIERARCHICAL_CLUSTER_H

#include "general_include.h"


// a feature idx binary tree
class idNode
{
public:
    set<int> idList;
    idNode* left;
    idNode* right;
    idNode* cur_root;
    idNode* parent;

    bool metaflag = false; // a flag for meta triper 3 features

    // initializations
    idNode()
    {
        left = NULL;
        right = NULL;
        cur_root = NULL;
        parent = NULL;
    }

    idNode(int id)
    {
        idList.insert(id);
        left = NULL;
        right = NULL;
        cur_root = NULL;
        parent = NULL;
    }

    // functions
    idNode* combine(idNode* l, idNode* r);
};

// combine two subtrees
idNode* combine(idNode* l, idNode* r, vector<idNode*> leaves)
{
    idNode* newGroup = new idNode();

    // combine two id sets
    newGroup->idList.insert(l->idList.begin(), l->idList.end());
    newGroup->idList.insert(r->idList.begin(), r->idList.end());

    newGroup->left = l;
    newGroup->right = r;
    l->parent = newGroup;
    r->parent = newGroup;

    l->cur_root = newGroup;
    r->cur_root = newGroup;

    // update the current root for each leaf
    /*for (auto iter = newGroup->idList.begin(); iter!= newGroup->idList.end();iter++)
    {
        leaves[*iter]->cur_root = newGroup;
    }*/

    return newGroup;
}

/*
// find the current root of some certain leaf
idNode* findRoot(idNode * node)
{
    if (node->cur_root)
    {
        return node->cur_root;
    }

    idNode * root = node;
    while (root->parent != NULL)
    {
        root = root->parent;
        node->cur_root = root;
    }
    return root;
}*/


// find the current root of some certain leaf
idNode* findRoot(idNode * node)
{
    idNode * root = node;
    idNode * sec = node;

    // if its cur_root is NULL, return itself
    if (root->cur_root == NULL)
        return root;

    while(root->cur_root != NULL)
    {
        root = root->cur_root;
    }

    while(sec->cur_root != NULL)
    {
        idNode* temp = sec->cur_root;
        sec->cur_root = root;
        sec = temp;
    }
    return root;
}

// find the common ancestor node
idNode* findAncestor(vector<idNode*> nodes)
{
    while(1)
    {
        idNode* parent = nodes[0]->parent;

    }

}


// free the tree
void freeHCTree(idNode * root)
{
    if (root)
    {
        freeHCTree(root->left);
        freeHCTree(root->right);
        delete root;
    }
}



#endif // HIERARCHICAL_CLUSTER_H
