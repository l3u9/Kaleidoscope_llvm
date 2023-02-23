#ifndef LLGM_EXECUTIONENGINE_ORC_KALEIDOSCOPEJIT_H
#define LLGM_EXECUTIONENGINE_ORC_KALEIDOSCOPEJIT_H

#include "llvm/ADT/StringRef.h"
#include "llvm/ExecutionEngine/JITSymbol.h"
#include "llvm/ExecutionEngine/Orc/CompileOnDemandLayer.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/ExecutionEngine/Orc/Core.h"
#include "llvm/ExecutionEngine/Orc/EPCIndirectionUtils.h"
#include "llvm/ExecutionEngine/Orc/ExecutionUtils.h"
#include "llvm/ExecutionEngine/Orc/ExecutorProcessControl.h"
#include "llvm/ExecutionEngine/Orc/IRCompileLayer.h"
#include "llvm/ExecutionEngine/Orc/IRTransformLayer.h"
#include "llvm/ExecutionEngine/Orc/JITTargetMachineBuilder.h"
#include "llvm/ExecutionEngine/Orc/RTDyldObjectLinkingLayer.h"
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include <memory>


class PrototypeAST;
class ExprAST;

class FunctionAST
{
    std::unique_ptr<PrototypeAST> Proto;
    std::unique_ptr<ExprAST> Body;

    public:
        FunctionAST(std::unique_ptr<PrototypeAST> Proto, std::unique_ptr<ExprAST> Body)
        : Proto(std::move(Proto)), Body(std::move(Body)) {}

        const PrototypeAST &getProto() const;
        const std::string &getName() const;
        llvm::Function *codegen();
};

llvm::orc::ThreadSafeModule irgenAndTakeOwnership(FunctionAST &FnAST, const std::string &Suffix);


namespace llvm {
namespace orc {

class KaleidoscopeASTLayer;
class KaleidoscopeJIT;

class KaleidoscopeASTMaterializationUnit : public MaterializationUnit
{
    public:
        KaleidoscopeASTMaterializationUnit(KaleidoscopeASTLayer &L, std::unique_ptr<FunctionAST> F);

        StringRef getName() const override 
        {
            return "KaleidoscopeASTMaterializationUnit";
        }

        void materialize(std::unique_ptr<MaterializationResponsibility> R) override;
    
    private:
        void discard(const JITDylib &JD, const SymbolStringPtr &Sym) override
        {
            llvm_unreachable("Kaleidoscope functions are not overridable");
        }

        KaleidoscopeASTLayer &L;
        std::unique_ptr<FunctionAST> F;
};

class KaleidoscopeASTLayer  
{
    public:
        KaleidoscopeASTLayer(IRLayer &BaseLayer, const DataLayout &DL) : BaseLayer(BaseLayer), DL(DL) {}

        Error add(ResourceTrackerSP RT, std::unique_ptr<FunctionAST> F)
        {
            return RT->getJITDylib().define(
                std::make_unique<KaleidoscopeASTMaterializationUnit>(*this, std::move(F)), RT
            );
        }

        void emit(std::unique_ptr<MaterializationResponsibility> MR, std::unique_ptr<FunctionAST> F)
        {
            BaseLayer.emit(std::move(MR), irgenAndTakeOwnership(*F, ""));
        }

        MaterializationUnit::Interface getInterface(FunctionAST &F)
        {
            MangleAndInterner Mangle(BaseLayer.getExecutionSession(), DL);
            SymbolFlagsMap Symbols;
            Symbols[Mangle(F.getName())] = JITSymbolFlags(JITSymbolFlags::Exported | JITSymbolFlags::Callable);
            return MaterializationUnit::Interface(std::move(Symbols), nullptr);
        }

    private:
        IRLayer &BaseLayer;
        const DataLayout &DL;
                
};

KaleidoscopeASTMaterializationUnit::KaleidoscopeASTMaterializationUnit(KaleidoscopeASTLayer &L, std::unique_ptr<FunctionAST> F)
    : MaterializationUnit(L.getInterface(*F)), L(L), F(std::move(F)) {}

void KaleidoscopeASTMaterializationUnit::materialize(std::unique_ptr<MaterializationResponsibility> R) 
{
  L.emit(std::move(R), std::move(F));
}

class KaleidoscopeJIT
{
    private:
        std::unique_ptr<ExecutionSession> ES;
        std::unique_ptr<EPCIndirectionUtils> EPCIU;

        DataLayout DL;
        MangleAndInterner Mangle;

        RTDyldObjectLinkingLayer ObjectLayer;
        IRCompileLayer CompileLayer;
        IRTransformLayer OptimizeLayer;
        CompileOnDemandLayer CODLayer;


        JITDylib &MainJD;
    
        static void handleLazyCallThroughError()
        {
            errs() << "LazyCallThrough error: Could not find function body";
            exit(1);
        }

    public:
        KaleidoscopeJIT(std::unique_ptr<ExecutionSession> ES, std::unique_ptr<EPCIndirectionUtils> EPCIU, JITTargetMachineBuilder JTMB, DataLayout DL)
        : ES(std::move(ES)), EPCIU(std::move(EPCIU)), DL(std::move(DL)), Mangle(*this->ES, this->DL), 
        ObjectLayer(*this->ES, []() {return std::make_unique<SectionMemoryManager>();}),
        CompileLayer(*this->ES, ObjectLayer, std::make_unique<ConcurrentIRCompiler>(std::move(JTMB))), OptimizeLayer(*this->ES, CompileLayer, optimizeModule),
        CODLayer(*this->ES, OptimizeLayer, this->EPCIU->getLazyCallThroughManager(), [this] {return this->EPCIU->createIndirectStubsManager();}),
        MainJD(this->ES->createBareJITDylib("<main>"))
        {
            MainJD.addGenerator(
                cantFail(DynamicLibrarySearchGenerator::GetForCurrentProcess(DL.getGlobalPrefix()))
            );
        }
        ~KaleidoscopeJIT()
        {
            if(auto Err = ES->endSession())
                ES->reportError(std::move(Err));
            if (auto Err = EPCIU->cleanup())
                ES->reportError(std::move(Err));
        }

        static Expected<std::unique_ptr<KaleidoscopeJIT>> Create()
        {
            auto EPC = SelfExecutorProcessControl::Create();
            if(!EPC)
                return EPC.takeError();

            auto ES = std::make_unique<ExecutionSession>(std::move(*EPC));

            auto EPCIU = EPCIndirectionUtils::Create(ES->getExecutorProcessControl());

            if(!EPCIU)
                return EPCIU.takeError();

            if(auto Err = setUpInProcessLCTMReentryViaEPCIU(**EPCIU))
                return std::move(Err);

            JITTargetMachineBuilder JTMB( ES->getExecutorProcessControl().getTargetTriple());

            auto DL = JTMB.getDefaultDataLayoutForTarget();
            if(!DL)
                return DL.takeError();
            
            return std::make_unique<KaleidoscopeJIT>(std::move(ES), std::move(*EPCIU), std::move(JTMB), std::move(*DL));
        }
        
        const DataLayout &getDataLayout() const {return DL;}

        JITDylib &getMainJITDylib() { return MainJD; }

        Error addModule(ThreadSafeModule TSM, ResourceTrackerSP RT = nullptr)
        {
            if(!RT)
                RT = MainJD.getDefaultResourceTracker();
            return CompileLayer.add(RT, std::move(TSM));
        }

        Expected<JITEvaluatedSymbol> lookup(StringRef Name)
        {
            return ES->lookup({&MainJD}, Mangle(Name.str()));
        }

    private:
        static Expected<ThreadSafeModule> optimizeModule(ThreadSafeModule TSM, const MaterializationResponsibility &R)
        {
            TSM.withModuleDo([](Module &M)
            {
                auto FPM = std::make_unique<legacy::FunctionPassManager>(&M);

                FPM->add(createInstructionCombiningPass());
                FPM->add(createReassociatePass());
                FPM->add(createGVNPass());
                FPM->add(createCFGSimplificationPass());
                FPM->doInitialization();

                for(auto &F : M)
                    FPM->run(F);
            });
            return std::move(TSM);
        }
};

}
}



#endif